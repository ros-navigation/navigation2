/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */

#include <memory>
#include <string>
#include <utility>
#include <algorithm>
#include <vector>
#include "nav2_amcl/amcl_node.hpp"
#include "nav2_util/pf/pf.hpp"  // pf_vector_t
#include "nav2_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_tasks/map_service_client.hpp"

// For transform support
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

// Allows AMCL to run from bag file
// #include <rosbag/bag.h>
// #include <rosbag/view.h>


using nav2_util::BeamModel;
using nav2_util::LikelihoodFieldModel;
using nav2_util::LikelihoodFieldModelProb;

using nav2_util::DifferentialMotionModel;
using nav2_util::OmniMotionModel;
using nav2_util::Laser;
using nav2_util::LaserData;

using namespace std::chrono_literals;

static const char scan_topic_[] = "scan";

#if NEW_UNIFORM_SAMPLING
std::vector<std::pair<int, int>> AmclNode::free_space_indices;
#endif

AmclNode::AmclNode()
: Node("amcl", nav2_util::get_node_options_default()),
  sent_first_transform_(false),
  latest_tf_valid_(false),
  map_(NULL),
  pf_(NULL),
  resample_count_(0),
  motionModel_(NULL),
  laser_(NULL),
  initial_pose_hyp_(NULL),
  first_map_received_(false),
  first_reconfigure_call_(true)
{
  RCLCPP_INFO(get_logger(), "Initializing AMCL");
  std::lock_guard<std::recursive_mutex> l(configuration_mutex_);

  node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

  initAmclParams();

  dynamic_param_client_ = std::make_unique<nav2_dynamic_params::DynamicParamsClient>(node_);

  createMotionModel();

  // updatePoseFromServer();
  initial_pose_received = false;

  tfb_.reset(new tf2_ros::TransformBroadcaster(node_));
  tf_.reset(new tf2_ros::Buffer(get_clock()));
  tfl_.reset(new tf2_ros::TransformListener(*tf_));

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 2;
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",
      custom_qos_profile);
  particlecloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud",
      custom_qos_profile);

  auto handle_global_localization_callback =
    [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response) -> void
    {
      globalLocalizationCallback(request_header, request, response);
    };
  global_loc_srv_ = create_service<std_srvs::srv::Empty>("global_localization",
      handle_global_localization_callback);

  auto handle_nomotion_update_callback =
    [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response) -> void
    {
      nomotionUpdateCallback(request_header, request, response);
    };
  nomotion_update_srv_ = create_service<std_srvs::srv::Empty>("request_nomotion_update",
      handle_nomotion_update_callback);

  auto handle_set_map_callback = [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<nav_msgs::srv::SetMap::Request> request,
      std::shared_ptr<nav_msgs::srv::SetMap::Response> response) -> void
    {
      setMapCallback(request_header, request, response);
    };
  set_map_srv_ = create_service<nav_msgs::srv::SetMap>("set_map", handle_set_map_callback);


  custom_qos_profile.depth = 1;
  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::msg::LaserScan>(this,
      scan_topic_, custom_qos_profile);
  laser_scan_filter_ =
    new tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>(*laser_scan_sub_,
      *tf_,
      odom_frame_id_,
      100,
      node_);
  laser_scan_filter_->registerCallback(std::bind(&AmclNode::laserReceived,
    this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose",
    std::bind(&AmclNode::initialPoseReceived, this, std::placeholders::_1));

  if (use_map_topic_) {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map",
        std::bind(&AmclNode::mapReceived, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to map topic.");
  } else {
    requestMap();  // TODO(mkhansen): This seems to hang indefinitely - see issue #330
  }
  m_force_update = false;

  dynamic_param_client_->add_parameters({"use_map_topic_", "first_map_only_", "save_pose_rate",
      "laser_min_range", "laser_max_range", "max_beams",
      "min_particles", "max_particles", "pf_err",
      "pf_z", "alpha1", "alpha2", "alpha3", "alpha4", "alpha5",
      "do_beamskip", "beam_skip_distance", "beam_skip_threshold",
      "beam_skip_error_threshold", "z_hit", "z_short", "z_max",
      "z_rand", "sigma_hit", "lambda_short",
      "laser_likelihood_max_dist", "laser_model_type",
      "robot_model_type", "update_min_d", "update_min_a",
      "odom_frame_id", "base_frame_id", "global_frame_id",
      "recovery_alpha_slow", "recovery_alpha_fast", "tf_broadcast"});
  dynamic_param_client_->set_callback(
    std::bind(&AmclNode::reconfigureCB, this));

  // 15s timer to warn on lack of receipt of laser scans, #5209
  laser_check_interval_ = 15s;
  check_laser_timer_ =
    create_wall_timer(laser_check_interval_, std::bind(&AmclNode::checkLaserReceived, this));
  RCLCPP_INFO(get_logger(), "AMCL Initialization complete");
}

AmclNode::~AmclNode()
{
  // delete dsrv_;
  freeMapDependentMemory();
  // delete laser_scan_filter_;
  delete laser_scan_sub_;
  // TODO(mhpanah): delete everything allocated in constructor
}

void AmclNode::savePoseToServer()
{
// TODO(mhpanah): Enable saving pose to parameter server.
/*
  // We need to apply the last transform to the latest odom pose to get
  // the latest map pose to store.  We'll take the covariance from
  // last_published_pose.
  tf2::Transform odom_pose_tf2;
  tf2::impl::Converter<true, false>::convert(latest_odom_pose_.pose, odom_pose_tf2);
  tf2::Transform map_pose = latest_tf_.inverse() * odom_pose_tf2;

  double yaw = tf2::getYaw(map_pose.getRotation());
  RCLCPP_DEBUG(get_logger(), "Saving pose to server. x: %.3f, y: %.3f",
    map_pose.getOrigin().x(), map_pose.getOrigin().y());

  parameters_client->set_parameters({
    rclcpp::Parameter("initial_pose_x", map_pose.getOrigin().x()),
    rclcpp::Parameter("initial_pose_y", map_pose.getOrigin().y()),
    rclcpp::Parameter("initial_pose_a", yaw),
    rclcpp::Parameter("initial_cov_xx",
    last_published_pose.pose.covariance[6 * 0 + 0]),
    rclcpp::Parameter("initial_cov_yy",
    last_published_pose.pose.covariance[6 * 1 + 1]),
    rclcpp::Parameter("initial_cov_aa",
    last_published_pose.pose.covariance[6 * 5 + 5]),
  });
*/
}

void AmclNode::updatePoseFromServer()
{
  init_pose_[0] = 0.0;
  init_pose_[1] = 0.0;
  init_pose_[2] = 0.0;
  init_cov_[0] = 0.5 * 0.5;
  init_cov_[1] = 0.5 * 0.5;
  init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);

  // TODO(mhpanah): Enable reading pose from parameter server.
/*
  // Check for NAN on input from param server, #5239
  double tmp_pos;

  tmp_pos = parameters_client->get_parameter("initial_pose_x", init_pose_[0]);

  if (!std::isnan(tmp_pos)) {
    init_pose_[0] = tmp_pos;
  } else {
    RCLCPP_WARN(get_logger(), "ignoring NAN in initial pose X position");
  }

  tmp_pos = parameters_client->get_parameter("initial_pose_y", init_pose_[1]);

  if (!std::isnan(tmp_pos)) {
    init_pose_[1] = tmp_pos;
  } else {
    RCLCPP_WARN(get_logger(), "ignoring NAN in initial pose Y position");
  }

  tmp_pos = parameters_client->get_parameter("initial_pose_a", init_pose_[2]);

  if (!std::isnan(tmp_pos)) {
    init_pose_[2] = tmp_pos;
  } else {
    RCLCPP_WARN(get_logger(), "ignoring NAN in initial pose Yaw");
  }

  tmp_pos = parameters_client->get_parameter("initial_cov_xx", init_cov_[0]);

  if (!std::isnan(tmp_pos)) {
    init_cov_[0] = tmp_pos;
  } else {
    RCLCPP_WARN(get_logger(), "ignoring NAN in initial covariance XX");
  }

  tmp_pos = parameters_client->get_parameter("initial_cov_yy", init_cov_[1]);

  if (!std::isnan(tmp_pos)) {
    init_cov_[1] = tmp_pos;
  } else {
    RCLCPP_WARN(get_logger(), "ignoring NAN in initial covariance YY");
  }

  tmp_pos = parameters_client->get_parameter("initial_cov_aa", init_cov_[2]);

  if (!std::isnan(tmp_pos)) {
    init_cov_[2] = tmp_pos;
  } else {
    RCLCPP_WARN(get_logger(), "ignoring NAN in initial covariance AA");
  }
  initial_pose_received = true;
*/
}

void
AmclNode::checkLaserReceived()
{
  if (last_laser_received_ts_.nanoseconds() == 0) {
    RCLCPP_WARN(
      get_logger(), "Laser scan has not been received"
      " (and thus no pose updates have been published)."
      " Verify that data is being published on the %s topic.", scan_topic_);
    return;
  }

  rclcpp::Duration d = this->now() - last_laser_received_ts_;
  if (d.nanoseconds() * 1e-9 > laser_check_interval_.count()) {
    RCLCPP_WARN(
      get_logger(), "No laser scan received (and thus no pose updates have been published) for %f"
      " seconds.  Verify that data is being published on the %s topic.",
      d.nanoseconds() * 1e-9,
      scan_topic_);
  }
}

void
AmclNode::requestMap()
{
  std::lock_guard<std::recursive_mutex> ml(configuration_mutex_);

  nav2_tasks::MapServiceClient map_client;
  map_client.wait_for_service(std::chrono::seconds(2));

  auto request = std::make_shared<nav2_tasks::MapServiceClient::MapServiceRequest>();

  RCLCPP_INFO(get_logger(), "AmclNode: Processing request map service.");
  auto result = map_client.invoke(request);

  handleMapMessage(result->map);
}

void
AmclNode::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "AmclNode: A new map was received.");
  if (first_map_only_ && first_map_received_) {
    return;
  }

  handleMapMessage(*msg);
  first_map_received_ = true;
}

void
AmclNode::handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg)
{
  std::lock_guard<std::recursive_mutex> cfl(configuration_mutex_);

  RCLCPP_INFO(get_logger(), "Received a %d X %d map @ %.3f m/pix\n",
    msg.info.width,
    msg.info.height,
    msg.info.resolution);
  if (msg.header.frame_id != global_frame_id_) {
    RCLCPP_WARN(
      get_logger(), "Frame_id of map received:'%s' doesn't match global_frame_id:'%s'. This could"
      " cause issues with reading published topics",
      msg.header.frame_id.c_str(),
      global_frame_id_.c_str());
  }

  freeMapDependentMemory();
  // Clear queued laser objects because they hold pointers to the existing
  // map, #5202.
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();

  map_ = convertMap(msg);

#if NEW_UNIFORM_SAMPLING
  // Index of free space
  free_space_indices.resize(0);
  for (int i = 0; i < map_->size_x; i++) {
    for (int j = 0; j < map_->size_y; j++) {
      if (map_->cells[MAP_INDEX(map_, i, j)].occ_state == -1) {
        free_space_indices.push_back(std::make_pair(i, j));
      }
    }
  }
#endif
  // Create the particle filter
  pf_ = pf_alloc(min_particles_, max_particles_,
      alpha_slow_, alpha_fast_,
      (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
      reinterpret_cast<void *>(map_));  // (void *)map_);
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter
  updatePoseFromServer();
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0];
  pf_init_pose_cov.m[1][1] = init_cov_[1];
  pf_init_pose_cov.m[2][2] = init_cov_[2];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;

  // Instantiate the sensor objects
  delete motionModel_;
  createMotionModel();

  // Laser
  delete laser_;
  createLaserObject();

  // In case the initial pose message arrived before the first map,
  // try to apply the initial pose now that the map has arrived.
  applyInitialPose();
}

void
AmclNode::freeMapDependentMemory()
{
  if (map_ != NULL) {
    map_free(map_);
    map_ = NULL;
  }

  if (pf_ != NULL) {
    pf_free(pf_);
    pf_ = NULL;
  }

  delete motionModel_;
  motionModel_ = NULL;

  delete laser_;
  laser_ = NULL;
}

/**
 * Convert an OccupancyGrid map message into the internal
 * representation. This allocates a map_t and returns it.
 */
map_t *
AmclNode::convertMap(const nav_msgs::msg::OccupancyGrid & map_msg)
{
  map_t * map = map_alloc();
  // ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells =
    reinterpret_cast<map_cell_t *>(malloc(sizeof(map_cell_t) * map->size_x * map->size_y));

  // ROS_ASSERT(map->cells);
  for (int i = 0; i < map->size_x * map->size_y; i++) {
    if (map_msg.data[i] == 0) {
      map->cells[i].occ_state = -1;
    } else if (map_msg.data[i] == 100) {
      map->cells[i].occ_state = +1;
    } else {
      map->cells[i].occ_state = 0;
    }
  }

  return map;
}

bool
AmclNode::getOdomPose(
  geometry_msgs::msg::PoseStamped & odom_pose,
  double & x, double & y, double & yaw,
  const rclcpp::Time & t, const std::string & f)
{
  // Get the robot's pose
  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = nav2_util::strip_leading_slash(f);
  ident.header.stamp = t;
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

  try {
    this->tf_->transform(ident, odom_pose, odom_frame_id_);
  } catch (tf2::TransformException e) {
    ++scan_error_count_;
    if (scan_error_count_ % 20 == 0) {
      RCLCPP_ERROR(
        get_logger(), "(%d) consecutive laser scan transforms failed: (%s)", scan_error_count_,
        e.what());
    }
    return false;
  }
  scan_error_count_ = 0;  // reset since we got a good transform
  x = odom_pose.pose.position.x;
  y = odom_pose.pose.position.y;
  yaw = tf2::getYaw(odom_pose.pose.orientation);

  return true;
}

pf_vector_t
AmclNode::uniformPoseGenerator(void * arg)
{
  map_t * map = reinterpret_cast<map_t *>(arg);

#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int, int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale) / 2.0 - map->origin_x;
  max_x = (map->size_x * map->scale) / 2.0 + map->origin_x;
  min_y = (map->size_y * map->scale) / 2.0 - map->origin_y;
  max_y = (map->size_y * map->scale) / 2.0 + map->origin_y;

  pf_vector_t p;

  RCLCPP_DEBUG(get_logger(), "Generating new uniform sample");
  for (;; ) {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i, j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if (MAP_VALID(map, i, j) && (map->cells[MAP_INDEX(map, i, j)].occ_state == -1)) {
      break;
    }
  }
#endif
  return p;
}

void
AmclNode::globalLocalizationCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  if (map_ == NULL) {
    return;
  }

  std::lock_guard<std::recursive_mutex> gl(configuration_mutex_);

  RCLCPP_INFO(get_logger(), "Initializing with uniform distribution");
  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
    reinterpret_cast<void *>(map_));
  RCLCPP_INFO(get_logger(), "Global initialisation done!");

  pf_init_ = false;
}

// force nomotion updates (amcl updating without requiring motion)
void
AmclNode::nomotionUpdateCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  m_force_update = true;
  RCLCPP_INFO(get_logger(), "Requesting no-motion update");
}

void
AmclNode::setMapCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav_msgs::srv::SetMap::Request> req,
  std::shared_ptr<nav_msgs::srv::SetMap::Response>/*res*/)
{
  handleMapMessage(req->map);
  handleInitialPoseMessage(req->initial_pose);
}

void
AmclNode::laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
{
  std::string laser_scan_frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
  last_laser_received_ts_ = now();
  if (map_ == NULL) {
    return;
  }
  if (!initial_pose_received) {
    return;
  }
  std::lock_guard<std::recursive_mutex> lr(configuration_mutex_);
  int laser_index = -1;
  geometry_msgs::msg::PoseStamped laser_pose;

  // Do we have the base->base_laser Tx yet?
  if (frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end()) {
    if (!addNewScanner(laser_index, laser_scan, laser_scan_frame_id, laser_pose)) {
      return;  // could not find transform
    }
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if (!getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
    laser_scan->header.stamp, base_frame_id_))
  {
    RCLCPP_ERROR(get_logger(), "Couldn't determine robot's pose associated with laser scan");
    return;
  }

  pf_vector_t delta = pf_vector_zero();
  bool force_publication = false;
  if (!pf_init_) {
    // Pose at last filter update
    pf_odom_pose_ = pose;
    pf_init_ = true;

    for (unsigned int i = 0; i < lasers_update_.size(); i++) {
      lasers_update_[i] = true;
    }

    force_publication = true;
    resample_count_ = 0;
  } else {
    // Set the laser update flags
    if (shouldUpdateFilter(pose, delta)) {
      for (unsigned int i = 0; i < lasers_update_.size(); i++) {
        lasers_update_[i] = true;
      }
    }
    if (lasers_update_[laser_index]) {
      motionModel_->odometryUpdate(pf_, pose, delta);
    }
    m_force_update = false;
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if (lasers_update_[laser_index]) {
    updateFilter(laser_index, laser_scan, pose);

    // Resample the particles
    if (!(++resample_count_ % resample_interval_)) {
      pf_update_resample(pf_);
      resampled = true;
    }

    pf_sample_set_t * set = pf_->sets + pf_->current_set;
    RCLCPP_DEBUG(get_logger(), "Num samples: %d\n", set->sample_count);

    if (!m_force_update) {
      publishParticleCloud(set);
    }
  }
  if (resampled || force_publication) {
    amcl_hyp_t max_weight_hyps;
    std::vector<amcl_hyp_t> hyps;
    int max_weight_hyp = -1;
    if (getMaxWeightHyp(hyps, max_weight_hyps, max_weight_hyp)) {
      publishAmclPose(laser_scan, hyps, max_weight_hyp);
      calculateMaptoOdomTransform(laser_scan, hyps, max_weight_hyp);

      if (tf_broadcast_ == true) {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        auto stamp = tf2_ros::fromMsg(laser_scan->header.stamp);
        tf2::TimePoint transform_expiration = stamp + transform_tolerance_;
        sendMapToOdomTransform(transform_expiration);
        sent_first_transform_ = true;
      }
    } else {
      RCLCPP_ERROR(get_logger(), "No pose!");
    }
  } else if (latest_tf_valid_) {
    if (tf_broadcast_ == true) {
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      tf2::TimePoint transform_expiration = tf2_ros::fromMsg(laser_scan->header.stamp) +
        transform_tolerance_;
      sendMapToOdomTransform(transform_expiration);
    }

    // Is it time to save our last pose to the param server
    tf2::TimePoint now = tf2_ros::fromMsg(this->now());
    if ((tf2::durationToSec(save_pose_period) > 0.0) &&
      (now - save_pose_last_time) >= save_pose_period)
    {
      this->savePoseToServer();
      save_pose_last_time = now;
    }
  }
}

bool AmclNode::addNewScanner(
  int & laser_index,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::string & laser_scan_frame_id,
  geometry_msgs::msg::PoseStamped & laser_pose)
{
  lasers_.push_back(createLaserObject());
  lasers_update_.push_back(true);
  laser_index = frame_to_laser_.size();

  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = laser_scan_frame_id;
  ident.header.stamp = rclcpp::Time();
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
  try {
    this->tf_->transform(ident, laser_pose, base_frame_id_);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Couldn't transform from %s to %s, "
      "even though the message notifier is in use",
      laser_scan->header.frame_id.c_str(),
      base_frame_id_.c_str());
    return false;
  }

  pf_vector_t laser_pose_v;
  laser_pose_v.v[0] = laser_pose.pose.position.x;
  laser_pose_v.v[1] = laser_pose.pose.position.y;
  // laser mounting angle gets computed later -> set to 0 here!
  laser_pose_v.v[2] = 0;
  lasers_[laser_index]->SetLaserPose(laser_pose_v);
  frame_to_laser_[laser_scan->header.frame_id] = laser_index;
  return true;
}

bool AmclNode::shouldUpdateFilter(const pf_vector_t pose, pf_vector_t & delta)
{
  delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
  delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
  delta.v[2] = angleutils::angle_diff(pose.v[2], pf_odom_pose_.v[2]);

  // See if we should update the filter
  bool update = fabs(delta.v[0]) > d_thresh_ ||
    fabs(delta.v[1]) > d_thresh_ ||
    fabs(delta.v[2]) > a_thresh_;
  update = update || m_force_update;
  return update;
}

bool AmclNode::updateFilter(
  const int & laser_index,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const pf_vector_t & pose)
{
  LaserData ldata;
  ldata.laser = lasers_[laser_index];
  ldata.range_count = laser_scan->ranges.size();
  // To account for lasers that are mounted upside-down, we determine the
  // min, max, and increment angles of the laser in the base frame.
  //
  // Construct min and max angles of laser, in the base_link frame.
  // Here we set the roll pich yaw of the lasers.  We assume roll and pich are zero.
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, laser_scan->angle_min);
  geometry_msgs::msg::QuaternionStamped min_q, inc_q;
  min_q.header.stamp = laser_scan->header.stamp;
  min_q.header.frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
  tf2::impl::Converter<false, true>::convert(q, min_q.quaternion);

  q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
  inc_q.header = min_q.header;
  tf2::impl::Converter<false, true>::convert(q, inc_q.quaternion);
  try {
    tf_->transform(min_q, min_q, base_frame_id_);
    tf_->transform(inc_q, inc_q, base_frame_id_);
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "Unable to transform min/max laser angles into base frame: %s",
      e.what());
    return false;
  }
  double angle_min = tf2::getYaw(min_q.quaternion);
  double angle_increment = tf2::getYaw(inc_q.quaternion) - angle_min;

  // wrapping angle to [-pi .. pi]
  angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;

  RCLCPP_DEBUG(
    get_logger(), "Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min,
    angle_increment);

  // Apply range min/max thresholds, if the user supplied them
  if (laser_max_range_ > 0.0) {
    ldata.range_max = std::min(laser_scan->range_max, static_cast<float>(laser_max_range_));
  } else {
    ldata.range_max = laser_scan->range_max;
  }
  double range_min;
  if (laser_min_range_ > 0.0) {
    range_min = std::max(laser_scan->range_min, static_cast<float>(laser_min_range_));
  } else {
    range_min = laser_scan->range_min;
  }

  // The LaserData destructor will free this memory
  ldata.ranges = new double[ldata.range_count][2];
  for (int i = 0; i < ldata.range_count; i++) {
    // amcl doesn't (yet) have a concept of min range.  So we'll map short
    // readings to max range.
    if (laser_scan->ranges[i] <= range_min) {
      ldata.ranges[i][0] = ldata.range_max;
    } else {
      ldata.ranges[i][0] = laser_scan->ranges[i];
    }
    // Compute bearing
    ldata.ranges[i][1] = angle_min +
      (i * angle_increment);
  }
  lasers_[laser_index]->sensorUpdate(pf_, reinterpret_cast<LaserData *>(&ldata));
  lasers_update_[laser_index] = false;
  pf_odom_pose_ = pose;
  return true;
}

void
AmclNode::publishParticleCloud(const pf_sample_set_t * set)
{
  geometry_msgs::msg::PoseArray cloud_msg;
  cloud_msg.header.stamp = this->now();
  cloud_msg.header.frame_id = global_frame_id_;
  cloud_msg.poses.resize(set->sample_count);
  for (int i = 0; i < set->sample_count; i++) {
    cloud_msg.poses[i].position.x = set->samples[i].pose.v[0];
    cloud_msg.poses[i].position.y = set->samples[i].pose.v[1];
    cloud_msg.poses[i].position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, set->samples[i].pose.v[2]);
    tf2::impl::Converter<false, true>::convert(q, cloud_msg.poses[i].orientation);
  }
  particlecloud_pub_->publish(cloud_msg);
}

bool
AmclNode::getMaxWeightHyp(
  std::vector<amcl_hyp_t> & hyps, amcl_hyp_t & max_weight_hyps,
  int & max_weight_hyp)
{
  // Read out the current hypotheses
  double max_weight = 0.0;
  hyps.resize(pf_->sets[pf_->current_set].cluster_count);
  for (int hyp_count = 0;
    hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
  {
    double weight;
    pf_vector_t pose_mean;
    pf_matrix_t pose_cov;
    if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov)) {
      RCLCPP_ERROR(get_logger(), "Couldn't get stats on cluster %d", hyp_count);
      return false;
    }

    hyps[hyp_count].weight = weight;
    hyps[hyp_count].pf_pose_mean = pose_mean;
    hyps[hyp_count].pf_pose_cov = pose_cov;

    if (hyps[hyp_count].weight > max_weight) {
      max_weight = hyps[hyp_count].weight;
      max_weight_hyp = hyp_count;
    }
  }

  if (max_weight > 0.0) {
    RCLCPP_DEBUG(get_logger(), "Max weight pose: %.3f %.3f %.3f",
      hyps[max_weight_hyp].pf_pose_mean.v[0],
      hyps[max_weight_hyp].pf_pose_mean.v[1],
      hyps[max_weight_hyp].pf_pose_mean.v[2]);

    max_weight_hyps = hyps[max_weight_hyp];
    return true;
  }
  return false;
}

void
AmclNode::publishAmclPose(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp)
{
  geometry_msgs::msg::PoseWithCovarianceStamped p;
  // Fill in the header
  p.header.frame_id = global_frame_id_;
  p.header.stamp = laser_scan->header.stamp;
  // Copy in the pose
  p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
  p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
  tf2::Quaternion q;
  q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
  tf2::impl::Converter<false, true>::convert(q, p.pose.pose.orientation);
  // Copy in the covariance, converting from 3-D to 6-D
  pf_sample_set_t * set = pf_->sets + pf_->current_set;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      // p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
      p.pose.covariance[6 * i + j] = set->cov.m[i][j];
    }
  }
  p.pose.covariance[6 * 5 + 5] = set->cov.m[2][2];
  float temp = 0.0;
  for (auto covariance_value : p.pose.covariance) {
    temp += covariance_value;
  }
  temp += p.pose.pose.position.x + p.pose.pose.position.y;
  if (!std::isnan(temp)) {
    RCLCPP_DEBUG(get_logger(), "AmclNode publishing pose");
    pose_pub_->publish(p);
    last_published_pose = p;
  } else {
    RCLCPP_WARN(get_logger(), "AMCL covariance or pose is NaN, likely due to an invalid "
      "configuration or faulty sensor measurements! Pose is not available!");
  }

  RCLCPP_DEBUG(get_logger(), "New pose: %6.3f %6.3f %6.3f",
    hyps[max_weight_hyp].pf_pose_mean.v[0],
    hyps[max_weight_hyp].pf_pose_mean.v[1],
    hyps[max_weight_hyp].pf_pose_mean.v[2]);
}

void
AmclNode::calculateMaptoOdomTransform(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp)
{
  // subtracting base to odom from map to base and send map to odom instead
  geometry_msgs::msg::PoseStamped odom_to_map;
  try {
    tf2::Quaternion q;
    q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
    tf2::Transform tmp_tf(q, tf2::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
      hyps[max_weight_hyp].pf_pose_mean.v[1],
      0.0));

    geometry_msgs::msg::PoseStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = base_frame_id_;
    tmp_tf_stamped.header.stamp = laser_scan->header.stamp;
    tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

    this->tf_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
  } catch (tf2::TransformException) {
    RCLCPP_DEBUG(get_logger(), "Failed to subtract base to odom transform");
    return;
  }

  tf2::impl::Converter<true, false>::convert(odom_to_map.pose, latest_tf_);
  latest_tf_valid_ = true;
}

void
AmclNode::sendMapToOdomTransform(const tf2::TimePoint & transform_expiration)
{
  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  tmp_tf_stamped.header.frame_id = global_frame_id_;
  tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
  tmp_tf_stamped.child_frame_id = odom_frame_id_;
  tf2::impl::Converter<false, true>::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
  this->tfb_->sendTransform(tmp_tf_stamped);
}

void
AmclNode::initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  handleInitialPoseMessage(*msg);
  initial_pose_received = true;
}

void
AmclNode::handleInitialPoseMessage(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  std::lock_guard<std::recursive_mutex> prl(configuration_mutex_);

  if (msg.header.frame_id == "") {
    // This should be removed at some point
    RCLCPP_WARN(
      get_logger(),
      "Received initial pose with empty frame_id. You should always supply a frame_id.");
  } else if (nav2_util::strip_leading_slash(msg.header.frame_id) != global_frame_id_) {
    RCLCPP_WARN(
      get_logger(), "Ignoring initial pose in frame \"%s\"; initial poses must be in the global"
      " frame, \"%s\"",
      nav2_util::strip_leading_slash(msg.header.frame_id).c_str(),
      global_frame_id_.c_str());
    return;
  }

  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  geometry_msgs::msg::TransformStamped tx_odom;
  try {
    // wait a little for the latest tf to become available
    tx_odom = tf_->lookupTransform(base_frame_id_, tf2_ros::fromMsg(msg.header.stamp),
        base_frame_id_, tf2::TimePoint(),
        odom_frame_id_);
  } catch (tf2::TransformException e) {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if (sent_first_transform_) {
      RCLCPP_WARN(get_logger(), "Failed to transform initial pose in time (%s)", e.what());
    }
    tf2::impl::Converter<false, true>::convert(tf2::Transform::getIdentity(), tx_odom.transform);
  }

  tf2::Transform tx_odom_tf2;
  tf2::impl::Converter<true, false>::convert(tx_odom.transform, tx_odom_tf2);
  tf2::Transform pose_old, pose_new;
  tf2::impl::Converter<true, false>::convert(msg.pose.pose, pose_old);
  pose_new = pose_old * tx_odom_tf2;

  // Transform into the global frame

  RCLCPP_INFO(get_logger(), "Setting pose (%.6f): %.3f %.3f %.3f",
    this->now().nanoseconds() * 1e-9,
    pose_new.getOrigin().x(),
    pose_new.getOrigin().y(),
    tf2::getYaw(pose_new.getRotation()));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = tf2::getYaw(pose_new.getRotation());
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      pf_init_pose_cov.m[i][j] = msg.pose.covariance[6 * i + j];
    }
  }

  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6 * 5 + 5];

  delete initial_pose_hyp_;
  initial_pose_hyp_ = new amcl_hyp_t();
  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;

  applyInitialPose();
}

void
AmclNode::applyInitialPose()
{
  std::lock_guard<std::recursive_mutex> cfl(configuration_mutex_);

  // If initial_pose_hyp_ and map_ are both non-null, apply the initial
  // pose to the particle filter state.

  if (initial_pose_hyp_ != nullptr && map_ != nullptr) {
    pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
    pf_init_ = false;
    delete initial_pose_hyp_;
    initial_pose_hyp_ = nullptr;
  }
}

nav2_util::Laser *
AmclNode::createLaserObject()
{
  if (map_ == NULL) {
    RCLCPP_WARN(get_logger(), "Map is not received yet.");
    return NULL;
  }
  if (sensor_model_type_ == "beam") {
    laser_ = new BeamModel(z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_,
        0.0, max_beams_, map_);
  } else if (sensor_model_type_ == "likelihood_field_prob") {
    laser_ = new LikelihoodFieldModelProb(z_hit_, z_rand_, sigma_hit_, laser_likelihood_max_dist_,
        do_beamskip_, beam_skip_distance_, beam_skip_threshold_,
        beam_skip_error_threshold_, max_beams_, map_);
  } else {
    laser_ = new LikelihoodFieldModel(z_hit_, z_rand_, sigma_hit_, laser_likelihood_max_dist_,
        max_beams_, map_);
  }
  return laser_;
}

void
AmclNode::createMotionModel()
{
  if (robot_model_type_ == "differential") {
    motionModel_ = new DifferentialMotionModel(alpha1_, alpha2_, alpha3_, alpha4_);
    RCLCPP_INFO(get_logger(), "Robot motion model is differential");
  } else if (robot_model_type_ == "omnidirectional") {
    motionModel_ = new OmniMotionModel(alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
    RCLCPP_INFO(get_logger(), "Robot motion model is omnidirectional");
  } else {
    RCLCPP_WARN(get_logger(), "Unknown robot motion model, defaulting to differential model");
    motionModel_ = new DifferentialMotionModel(alpha1_, alpha2_, alpha3_, alpha4_);
  }
}

void
AmclNode::initAmclParams()
{
  // Grab params off the param server
  get_parameter_or_set("use_map_topic_", use_map_topic_, true);  // When false AMCL hangs
                                                                 // in constructor (issue #330)
  get_parameter_or_set("first_map_only_", first_map_only_, true);
  double save_pose_rate;
  get_parameter_or_set("save_pose_rate", save_pose_rate, 0.5);
  save_pose_period = tf2::durationFromSec(1.0 / save_pose_rate);
  get_parameter_or_set("laser_min_range", laser_min_range_, -1.0);
  get_parameter_or_set("laser_max_range", laser_max_range_, 100.0);
  get_parameter_or_set("max_beams", max_beams_, 60);
  get_parameter_or_set("min_particles", min_particles_, 500);
  get_parameter_or_set("max_particles", max_particles_, 2000);
  get_parameter_or_set("pf_err", pf_err_, 0.05);
  get_parameter_or_set("pf_z", pf_z_, 0.99);
  get_parameter_or_set("alpha1", alpha1_, 0.2);
  get_parameter_or_set("alpha2", alpha2_, 0.2);
  get_parameter_or_set("alpha3", alpha3_, 0.2);
  get_parameter_or_set("alpha4", alpha4_, 0.2);
  get_parameter_or_set("alpha5", alpha5_, 0.2);
  get_parameter_or_set("do_beamskip", do_beamskip_, false);
  get_parameter_or_set("beam_skip_distance", beam_skip_distance_, 0.5);
  get_parameter_or_set("beam_skip_threshold", beam_skip_threshold_, 0.3);
  get_parameter_or_set("beam_skip_error_threshold", beam_skip_error_threshold_, 0.9);
  get_parameter_or_set("z_hit", z_hit_, 0.5);
  get_parameter_or_set("z_short", z_short_, 0.05);
  get_parameter_or_set("z_max", z_max_, 0.05);
  get_parameter_or_set("z_rand", z_rand_, 0.5);
  get_parameter_or_set("sigma_hit", sigma_hit_, 0.2);
  get_parameter_or_set("lambda_short", lambda_short_, 0.1);
  get_parameter_or_set("laser_likelihood_max_dist", laser_likelihood_max_dist_, 2.0);
  get_parameter_or_set("laser_model_type", sensor_model_type_, std::string("likelihood_field"));
  RCLCPP_INFO(get_logger(), "Sensor model type is: \"%s\"", sensor_model_type_.c_str());
  get_parameter_or_set("robot_model_type", robot_model_type_, std::string("differential"));
  get_parameter_or_set("update_min_d", d_thresh_, 0.25);
  get_parameter_or_set("update_min_a", a_thresh_, 0.2);
  get_parameter_or_set("odom_frame_id", odom_frame_id_, std::string("odom"));
  get_parameter_or_set("base_frame_id", base_frame_id_, std::string("base_footprint"));
  get_parameter_or_set("global_frame_id", global_frame_id_, std::string("map"));
  get_parameter_or_set("resample_interval", resample_interval_, 1);
  double tmp_tol;
  get_parameter_or_set("transform_tolerance", tmp_tol, 1.0);
  transform_tolerance_ = tf2::durationFromSec(tmp_tol);
  get_parameter_or_set("recovery_alpha_slow", alpha_slow_, 0.0);
  get_parameter_or_set("recovery_alpha_fast", alpha_fast_, 0.0);
  get_parameter_or_set("tf_broadcast", tf_broadcast_, true);
  odom_frame_id_ = nav2_util::strip_leading_slash(odom_frame_id_);
  base_frame_id_ = nav2_util::strip_leading_slash(base_frame_id_);
  global_frame_id_ = nav2_util::strip_leading_slash(global_frame_id_);
}

void
AmclNode::reconfigureCB()
{
// TODO(mhpanah): restore defaults
/*
  if (config.restore_defaults) {
    config = default_config_;
    // avoid looping
    config.restore_defaults = false;
  }
*/
  dynamic_param_client_->get_event_param("update_min_d", d_thresh_);
  dynamic_param_client_->get_event_param("update_min_a", a_thresh_);
  dynamic_param_client_->get_event_param("resample_interval", resample_interval_);
  dynamic_param_client_->get_event_param("laser_min_range", laser_min_range_);
  dynamic_param_client_->get_event_param("laser_max_range", laser_max_range_);

  double save_pose_rate;
  dynamic_param_client_->get_event_param("save_pose_rate", save_pose_rate);
  save_pose_period = tf2::durationFromSec(1.0 / save_pose_rate);

  double tmp_tol;
  dynamic_param_client_->get_event_param("transform_tolerance", tmp_tol);
  transform_tolerance_ = tf2::durationFromSec(tmp_tol);

  dynamic_param_client_->get_event_param("laser_max_beams", max_beams_);
  dynamic_param_client_->get_event_param("odom_alpha1", alpha1_);
  dynamic_param_client_->get_event_param("odom_alpha2", alpha2_);
  dynamic_param_client_->get_event_param("odom_alpha3", alpha3_);
  dynamic_param_client_->get_event_param("odom_alpha4", alpha4_);
  dynamic_param_client_->get_event_param("odom_alpha5", alpha5_);
  dynamic_param_client_->get_event_param("laser_z_hit", z_hit_);
  dynamic_param_client_->get_event_param("laser_z_short", z_short_);
  dynamic_param_client_->get_event_param("laser_z_max", z_max_);
  dynamic_param_client_->get_event_param("laser_z_rand", z_rand_);
  dynamic_param_client_->get_event_param("laser_sigma_hit", sigma_hit_);
  dynamic_param_client_->get_event_param("laser_lambda_short", lambda_short_);
  dynamic_param_client_->get_event_param("laser_likelihood_max_dist",
    laser_likelihood_max_dist_);
  dynamic_param_client_->get_event_param("laser_model_type", sensor_model_type_);
  dynamic_param_client_->get_event_param("robot_model_type", robot_model_type_);

  dynamic_param_client_->get_event_param("min_particles", min_particles_);
  dynamic_param_client_->get_event_param("max_particles", max_particles_);

  if (min_particles_ > max_particles_) {
    RCLCPP_WARN(get_logger(), "You've set min_particles to be greater than max particles,"
      " this isn't allowed so they'll be set to be equal.");
    max_particles_ = min_particles_;
  }
  dynamic_param_client_->get_event_param("recovery_alpha_slow", alpha_slow_);
  dynamic_param_client_->get_event_param("recovery_alpha_fast", alpha_fast_);
  dynamic_param_client_->get_event_param("tf_broadcast", tf_broadcast_);

  dynamic_param_client_->get_event_param("do_beamskip", do_beamskip_);
  dynamic_param_client_->get_event_param("beam_skip_distance", beam_skip_distance_);
  dynamic_param_client_->get_event_param("beam_skip_threshold", beam_skip_threshold_);

  if (pf_ != NULL) {
    pf_free(pf_);
    pf_ = NULL;
  }
  pf_ = pf_alloc(min_particles_, max_particles_, alpha_slow_, alpha_fast_,
      (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
      reinterpret_cast<void *>(map_));

  dynamic_param_client_->get_event_param("kld_err", pf_err_);
  dynamic_param_client_->get_event_param("kld_z", pf_z_);
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;
  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = last_published_pose.pose.pose.position.x;
  pf_init_pose_mean.v[1] = last_published_pose.pose.pose.position.y;
  pf_init_pose_mean.v[2] = tf2::getYaw(last_published_pose.pose.pose.orientation);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = last_published_pose.pose.covariance[6 * 0 + 0];
  pf_init_pose_cov.m[1][1] = last_published_pose.pose.covariance[6 * 1 + 1];
  pf_init_pose_cov.m[2][2] = last_published_pose.pose.covariance[6 * 5 + 5];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;

// Instantiate robot motion model and sensor object
  // Odometry
  delete motionModel_;
  createMotionModel();
  // Laser
  delete laser_;
  createLaserObject();
}
