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

#include <boost/foreach.hpp>
#include <memory>
#include <string>
#include <utility>
#include <algorithm>
#include <vector>
#include "amcl_node.hpp"
#include "nav2_util/pf/pf.h"  // pf_vector_t
#include "nav2_util/strutils.hpp"
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
// #include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

// Dynamic_reconfigure
// #include "dynamic_reconfigure/server.h"

// Allows AMCL to run from bag file
// #include <rosbag/bag.h>
// #include <rosbag/view.h>

using amcl::LASER_MODEL_BEAM;
using amcl::LASER_MODEL_LIKELIHOOD_FIELD;
using amcl::LASER_MODEL_LIKELIHOOD_FIELD_PROB;
using amcl::ODOM_MODEL_DIFF;
using amcl::Odom;
using amcl::Laser;
using amcl::ODOM_MODEL_OMNI;
using amcl::OdomData;
using amcl::SensorData;
using amcl::LaserData;

using namespace std::chrono_literals;
static const auto TRANSFORM_TIMEOUT = 1s;

static double
normalize(double z)
{
  return atan2(sin(z), cos(z));
}

static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a - b;
  d2 = 2 * M_PI - fabs(d1);
  if (d1 > 0) {
    d2 *= -1.0;
  }
  if (fabs(d1) < fabs(d2)) {
    return d1;
  } else {
    return d2;
  }
}

static const char scan_topic_[] = "scan";

#if NEW_UNIFORM_SAMPLING
std::vector<std::pair<int, int>> AmclNode::free_space_indices;
#endif

AmclNode::AmclNode()
: Node("amcl"),
  sent_first_transform_(false),
  latest_tf_valid_(false),
  map_(NULL),
  pf_(NULL),
  resample_count_(0),
  odom_(NULL),
  laser_(NULL),
  initial_pose_hyp_(NULL),
  first_map_received_(false),
  first_reconfigure_call_(true)
{
  std::lock_guard<std::recursive_mutex> l(configuration_mutex_);

  parameters_node_ = rclcpp::Node::make_shared("ParametersNode");
  parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(std::shared_ptr<rclcpp::Node>(parameters_node_));

  // Grab params off the param server
  use_map_topic_ = parameters_client->get_parameter("use_map_topic_", false);
  first_map_only_ = parameters_client->get_parameter("first_map_only_", false);

  double tmp;
  tmp = parameters_client->get_parameter("gui_publish_rate", 10.0);
  gui_publish_period = tf2::durationFromSec(1.0 / tmp);
  tmp = parameters_client->get_parameter("save_pose_rate", 0.5);
  save_pose_period = tf2::durationFromSec(1.0 / tmp);
  laser_min_range_ = parameters_client->get_parameter("laser_min_range", -1.0);
  laser_max_range_ = parameters_client->get_parameter("laser_max_range", 12.0);
  max_beams_ = parameters_client->get_parameter("max_beams", 60);
  min_particles_ = parameters_client->get_parameter("min_particles", 500);
  max_particles_ = parameters_client->get_parameter("max_particles", 2000);
  pf_err_ = parameters_client->get_parameter("pf_err", 0.05);
  pf_z_ = parameters_client->get_parameter("pf_z", 0.99);
  alpha1_ = parameters_client->get_parameter("alpha1", 0.2);
  alpha2_ = parameters_client->get_parameter("alpha2", 0.2);
  alpha3_ = parameters_client->get_parameter("alpha3", 0.2);
  alpha4_ = parameters_client->get_parameter("alpha4", 0.2);
  alpha5_ = parameters_client->get_parameter("alpha5", 0.2);
  do_beamskip_ = parameters_client->get_parameter("do_beamskip", false);
  beam_skip_distance_ = parameters_client->get_parameter("beam_skip_distance", 0.5);
  beam_skip_threshold_ = parameters_client->get_parameter("beam_skip_threshold", 0.3);
  beam_skip_error_threshold_ = parameters_client->get_parameter("beam_skip_error_threshold", 0.9);
  z_hit_ = parameters_client->get_parameter("z_hit", 0.5);
  z_short_ = parameters_client->get_parameter("z_short", 0.05);
  z_max_ = parameters_client->get_parameter("z_max", 0.05);
  z_rand_ = parameters_client->get_parameter("z_rand", 0.5);
  sigma_hit_ = parameters_client->get_parameter("sigma_hit", 0.2);
  lambda_short_ = parameters_client->get_parameter("lambda_short", 0.1);
  laser_likelihood_max_dist_ = parameters_client->get_parameter("laser_likelihood_max_dist", 2.0);

  std::string tmp_model_type;
  tmp_model_type =
    parameters_client->get_parameter("laser_model_type", std::string("likelihood_field"));
  if (tmp_model_type == "beam") {
    laser_model_type_ = LASER_MODEL_BEAM;
  } else if (tmp_model_type == "likelihood_field") {
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  } else if (tmp_model_type == "likelihood_field_prob") {
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  } else {
    RCLCPP_WARN(
      get_logger(), "Unknown laser model type \"%s\"; defaulting to likelihood_field model",
      tmp_model_type.c_str());
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  }

  tmp_model_type = parameters_client->get_parameter("tmp_model_type", std::string("differential"));
  if (tmp_model_type == "differential") {
    odom_model_type_ = ODOM_MODEL_DIFF;
  } else if (tmp_model_type == "omnidirectional") {
    odom_model_type_ = ODOM_MODEL_OMNI;
  } else {
    RCLCPP_WARN(get_logger(), "Unknown odom model type \"%s\"; defaulting to differential model",
      tmp_model_type.c_str());
    odom_model_type_ = ODOM_MODEL_DIFF;
  }

  d_thresh_ = parameters_client->get_parameter("update_min_d", 0.25);
  a_thresh_ = parameters_client->get_parameter("update_min_a", 0.2);
  odom_frame_id_ = parameters_client->get_parameter("odom_frame_id", std::string("odom"));
  base_frame_id_ = parameters_client->get_parameter("base_frame_id", std::string("base_footprint"));
  global_frame_id_ = parameters_client->get_parameter("global_frame_id", std::string("map"));
  resample_interval_ = parameters_client->get_parameter("resample_interval", 1);
  double tmp_tol;
  tmp_tol = parameters_client->get_parameter("transform_tolerance", 1.0);
  alpha_slow_ = parameters_client->get_parameter("recovery_alpha_slow", 0.0);
  alpha_fast_ = parameters_client->get_parameter("recovery_alpha_fast", 0.0);
  tf_broadcast_ = parameters_client->get_parameter("tf_broadcast", true);

  transform_tolerance_ = tf2::durationFromSec(tmp_tol);

  {
    double bag_scan_period;
    bag_scan_period = parameters_client->get_parameter("bag_scan_period", -1.0);
    bag_scan_period_ = std::chrono::duration<double>{bag_scan_period};
  }

  odom_frame_id_ = strutils::stripLeadingSlash(odom_frame_id_);
  base_frame_id_ = strutils::stripLeadingSlash(base_frame_id_);
  global_frame_id_ = strutils::stripLeadingSlash(global_frame_id_);

  updatePoseFromServer();

  cloud_pub_interval = std::chrono::duration<double>{1.0};

  tfb_.reset(new tf2_ros::TransformBroadcaster(parameters_node_));
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


  custom_qos_profile.depth = 100;
  // laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::msg::LaserScan>(this,
  //                   scan_topic_, custom_qos_profile);
  // Disabling laser_scan_filter
  /*
  laser_scan_filter_ =
          new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                             *tf_,
                                                             odom_frame_id_,
                                                             100,
                                                             nh_);
  laser_scan_filter_->registerCallback(std::bind(&AmclNode::laserReceived,
                                                   this, std::placeholders::_1));
  */
  laser_scan_filter_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_,
      std::bind(&AmclNode::laserReceived, this, std::placeholders::_1), custom_qos_profile);

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose",
    std::bind(&AmclNode::initialPoseReceived, this, std::placeholders::_1));

  if (use_map_topic_) {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("occ_grid",
        std::bind(&AmclNode::mapReceived, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to map topic.");
  } else {
    requestMap();
  }
  m_force_update = false;

#if 0
  dsrv_ = new dynamic_reconfigure::Server<amcl::AMCLConfig>(ros::NodeHandle("~"));
  dynamic_reconfigure::Server<amcl::AMCLConfig>::CallbackType cb = std::bind(
    &AmclNode::reconfigureCB, this, std::placeholders::_1, std::placeholders::_2);
  dsrv_->setCallback(cb);
#endif

  // 15s timer to warn on lack of receipt of laser scans, #5209
  laser_check_interval_ = 15s;
  check_laser_timer_ =
    create_wall_timer(laser_check_interval_, std::bind(&AmclNode::checkLaserReceived, this));
}

AmclNode::~AmclNode()
{
  // delete dsrv_;
  freeMapDependentMemory();
  // delete laser_scan_filter_;
  delete laser_scan_sub_;
  // TODO(mhpanah): delete everything allocated in constructor
}

#if 0
void AmclNode::reconfigureCB(AMCLConfig & config, uint32_t level)
{
  std::lock_gaurd<std::recursive_mutex> cfl(configuration_mutex_);

  // we don't want to do anything on the first call
  // which corresponds to startup
  if (first_reconfigure_call_) {
    first_reconfigure_call_ = false;
    default_config_ = config;
    return;
  }

  if (config.restore_defaults) {
    config = default_config_;
    // avoid looping
    config.restore_defaults = false;
  }

  d_thresh_ = config.update_min_d;
  a_thresh_ = config.update_min_a;

  resample_interval_ = config.resample_interval;

  laser_min_range_ = config.laser_min_range;
  laser_max_range_ = config.laser_max_range;

  gui_publish_period = ros::Duration(1.0 / config.gui_publish_rate);
  save_pose_period = ros::Duration(1.0 / config.save_pose_rate);

  transform_tolerance_.fromSec(config.transform_tolerance);

  max_beams_ = config.laser_max_beams;
  alpha1_ = config.odom_alpha1;
  alpha2_ = config.odom_alpha2;
  alpha3_ = config.odom_alpha3;
  alpha4_ = config.odom_alpha4;
  alpha5_ = config.odom_alpha5;

  z_hit_ = config.laser_z_hit;
  z_short_ = config.laser_z_short;
  z_max_ = config.laser_z_max;
  z_rand_ = config.laser_z_rand;
  sigma_hit_ = config.laser_sigma_hit;
  lambda_short_ = config.laser_lambda_short;
  laser_likelihood_max_dist_ = config.laser_likelihood_max_dist;

  if (config.laser_model_type == "beam") {
    laser_model_type_ = LASER_MODEL_BEAM;
  } else if (config.laser_model_type == "likelihood_field") {
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  } else if (config.laser_model_type == "likelihood_field_prob") {
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  }

  if (config.odom_model_type == "differential") {
    odom_model_type_ = ODOM_MODEL_DIFF;
  } else if (config.odom_model_type == "omnidirectional") {
    odom_model_type_ = ODOM_MODEL_OMNI;
  }

  if (config.min_particles > config.max_particles) {
    ROS_WARN(
      "You've set min_particles to be greater than max particles, this isn't allowed so they'll"
      " be set to be equal.");
    config.max_particles = config.min_particles;
  }

  min_particles_ = config.min_particles;
  max_particles_ = config.max_particles;
  alpha_slow_ = config.recovery_alpha_slow;
  alpha_fast_ = config.recovery_alpha_fast;
  tf_broadcast_ = config.tf_broadcast;

  do_beamskip_ = config.do_beamskip;
  beam_skip_distance_ = config.beam_skip_distance;
  beam_skip_threshold_ = config.beam_skip_threshold;

  if (pf_ != NULL) {
    pf_free(pf_);
    pf_ = NULL;
  }
  pf_ = pf_alloc(min_particles_, max_particles_,
      alpha_slow_, alpha_fast_,
      (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
      reinterpret_cast<void *>(map_));
  pf_err_ = config.kld_err;
  pf_z_ = config.kld_z;
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

  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new Odom();
  ROS_ASSERT(odom_);
  odom_->SetModel(odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
  // Laser
  delete laser_;
  laser_ = new Laser(max_beams_, map_);
  ROS_ASSERT(laser_);
  if (laser_model_type_ == LASER_MODEL_BEAM) {
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
      sigma_hit_, lambda_short_, 0.0);
  } else if (laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB) {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
      laser_likelihood_max_dist_,
      do_beamskip_, beam_skip_distance_,
      beam_skip_threshold_, beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model with probabilities.");
  } else if (laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD) {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
      laser_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }

  odom_frame_id_ = strutils::stripLeadingSlash(config.odom_frame_id);
  base_frame_id_ = strutils::stripLeadingSlash(config.base_frame_id);
  global_frame_id_ = strutils::stripLeadingSlash(config.global_frame_id);

  // Disabling laser_scan_filter
  /*
  delete laser_scan_filter_;
  laser_scan_filter_ =
          new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                             *tf_,
                                                             odom_frame_id_,
                                                             100,
                                                             nh_);
  laser_scan_filter_->registerCallback(std::bind(&AmclNode::laserReceived,
                                                   this, std::placeholders::_1));
  */
  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);
}
#endif


void AmclNode::runFromBag(const std::string & /*in_bag_fn*/)
{
#if 0
  rosbag::Bag bag;
  bag.open(in_bag_fn, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("tf"));
  std::string scan_topic_name = "base_scan";  // TODO(?): determine what topic this actually is
  topics.push_back(scan_topic_name);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  ros::Publisher laser_pub = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_name, 100);
  rclcpp::Publisher laser_pub = this->create_publisher<sensor_msgs::LaserScan>(scan_topic_name);
  ros::Publisher tf_pub = nh_.advertise<tf2_msgs::TFMessage>("/tf", 100);
  // Sleep for a second to let all subscribers connect
  ros::WallDuration(1.0).sleep();

  ros::WallTime start(ros::WallTime::now());

  // Wait for map
  while (ros::ok()) {
    {
      std::lock_guard<std::recursive_mutex> cfl(configuration_mutex_);
      if (map_) {
        ROS_INFO("Map is ready");
        break;
      }
    }
    ROS_INFO("Waiting for map...");
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1.0));
  }

  BOOST_FOREACH(rosbag::MessageInstance const msg, view)
  {
    if (!ros::ok()) {
      break;
    }

    // Process any ros messages or callbacks at this point
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration());

    tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
    if (tf_msg != NULL) {
      tf_pub.publish(msg);
      for (size_t ii = 0; ii < tf_msg->transforms.size(); ++ii) {
        tf_->setTransform(tf_msg->transforms[ii], "rosbag_authority");
      }
      continue;
    }

    sensor_msgs::LaserScan::ConstPtr base_scan = msg.instantiate<sensor_msgs::LaserScan>();
    if (base_scan != NULL) {
      laser_pub.publish(msg);
      // Disabling laser_scan_filter
      // laser_scan_filter_->add(base_scan);
      if (bag_scan_period_ > ros::WallDuration(0)) {
        bag_scan_period_.sleep();
      }
      continue;
    }

    ROS_WARN_STREAM("Unsupported message type" << msg.getTopic());
  }

  bag.close();

  double runtime = (ros::WallTime::now() - start).toSec();
  ROS_INFO("Bag complete, took %.1f seconds to process, shutting down", runtime);

  const geometry_msgs::Quaternion & q(last_published_pose.pose.pose.orientation);
  double yaw, pitch, roll;
  tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getEulerYPR(yaw, pitch, roll);
  ROS_INFO("Final location %.3f, %.3f, %.3f with stamp=%f",
    last_published_pose.pose.pose.position.x,
    last_published_pose.pose.pose.position.y,
    yaw, last_published_pose.header.stamp.toSec()
  );

  ros::shutdown();
#endif
}

void AmclNode::savePoseToServer()
{
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
}

void AmclNode::updatePoseFromServer()
{
  init_pose_[0] = 0.0;
  init_pose_[1] = 0.0;
  init_pose_[2] = 0.0;
  init_cov_[0] = 0.5 * 0.5;
  init_cov_[1] = 0.5 * 0.5;
  init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);

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
  map_client.waitForService(std::chrono::seconds(2));

  auto request = std::make_shared<nav2_tasks::MapServiceClient::MapServiceRequest>();

  RCLCPP_INFO(get_logger(), "AmclNode: Processing request map service.");
  auto result = map_client.invoke(request);

  handleMapMessage(result->map);
}

void
AmclNode::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "AmclNode: A new map was received.");
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
  // Odometry
  delete odom_;
  odom_ = new Odom();
  assert(odom_);
  odom_->SetModel(odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
  // Laser
  delete laser_;
  laser_ = new Laser(max_beams_, map_);
  assert(laser_);
  if (laser_model_type_ == LASER_MODEL_BEAM) {
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
      sigma_hit_, lambda_short_, 0.0);
  } else if (laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB) {
    RCLCPP_INFO(
      get_logger(),
      "Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
      laser_likelihood_max_dist_,
      do_beamskip_, beam_skip_distance_,
      beam_skip_threshold_, beam_skip_error_threshold_);
    RCLCPP_INFO(get_logger(), "Done initializing likelihood field model.");
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
      laser_likelihood_max_dist_);
    RCLCPP_INFO(get_logger(), "Done initializing likelihood field model.");
  }

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

  delete odom_;
  odom_ = NULL;

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
  ident.header.frame_id = strutils::stripLeadingSlash(f);
  ident.header.stamp = t;
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

  try {
    this->tf_->transform(ident, odom_pose, odom_frame_id_, TRANSFORM_TIMEOUT);
  } catch (tf2::TransformException e) {
    RCLCPP_WARN(get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }

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
  std::string laser_scan_frame_id = strutils::stripLeadingSlash(laser_scan->header.frame_id);
  last_laser_received_ts_ = now();
  if (map_ == NULL) {
    return;
  }

  std::lock_guard<std::recursive_mutex> lr(configuration_mutex_);
  int laser_index = -1;

  // Do we have the base->base_laser Tx yet?
  if (frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end()) {
    RCLCPP_DEBUG(get_logger(), "Setting up laser %d (frame_id=%s)\n",
      (int)frame_to_laser_.size(), laser_scan_frame_id.c_str());
    lasers_.push_back(new Laser(*laser_));
    lasers_update_.push_back(true);
    laser_index = frame_to_laser_.size();

    geometry_msgs::msg::PoseStamped ident;
    ident.header.frame_id = laser_scan_frame_id;
    ident.header.stamp = rclcpp::Time();
    tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

    geometry_msgs::msg::PoseStamped laser_pose;
    try {
      this->tf_->transform(ident, laser_pose, base_frame_id_, TRANSFORM_TIMEOUT);
    } catch (tf2::TransformException & e) {
      RCLCPP_ERROR(get_logger(), "Couldn't transform from %s to %s, "
        "even though the message notifier is in use",
        laser_scan->header.frame_id.c_str(),
        base_frame_id_.c_str());
      return;
    }

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.pose.position.x;
    laser_pose_v.v[1] = laser_pose.pose.position.y;
    // laser mounting angle gets computed later -> set to 0 here!
    laser_pose_v.v[2] = 0;
    lasers_[laser_index]->SetLaserPose(laser_pose_v);
    RCLCPP_DEBUG(get_logger(), "Received laser's pose wrt robot: %.3f %.3f %.3f",
      laser_pose_v.v[0],
      laser_pose_v.v[1],
      laser_pose_v.v[2]);

    frame_to_laser_[laser_scan->header.frame_id] = laser_index;
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

  if (pf_init_) {
    // Compute change in pose
    // delta = pf_vector_coord_sub(pose, pf_odom_pose_);
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

    // See if we should update the filter
    bool update = fabs(delta.v[0]) > d_thresh_ ||
      fabs(delta.v[1]) > d_thresh_ ||
      fabs(delta.v[2]) > a_thresh_;
    update = update || m_force_update;
    m_force_update = false;

    // Set the laser update flags
    if (update) {
      for (unsigned int i = 0; i < lasers_update_.size(); i++) {
        lasers_update_[i] = true;
      }
    }
  }

  bool force_publication = false;
  if (!pf_init_) {
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is now initialized
    pf_init_ = true;

    // Should update sensor data
    for (unsigned int i = 0; i < lasers_update_.size(); i++) {
      lasers_update_[i] = true;
    }

    force_publication = true;

    resample_count_ = 0;
  } else if (pf_init_ && lasers_update_[laser_index]) {  // If the robot has moved update the filter
    // printf("pose\n");
    // pf_vector_fprintf(pose, stdout, "%.3f");

    OdomData odata;
    odata.pose = pose;
    // HACK
    // Modify the delta in the action data so the filter gets
    // updated correctly
    odata.delta = delta;

    // Use the action data to update the filter
    odom_->UpdateAction(pf_, reinterpret_cast<SensorData *>(&odata));

    // Pose at last filter update
    // this->pf_odom_pose = pose;
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if (lasers_update_[laser_index]) {
    LaserData ldata;
    ldata.sensor = lasers_[laser_index];
    ldata.range_count = laser_scan->ranges.size();

    // To account for lasers that are mounted upside-down, we determine the
    // min, max, and increment angles of the laser in the base frame.
    //
    // Construct min and max angles of laser, in the base_link frame.
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, laser_scan->angle_min);
    geometry_msgs::msg::QuaternionStamped min_q, inc_q;
    min_q.header.stamp = laser_scan->header.stamp;
    min_q.header.frame_id = strutils::stripLeadingSlash(laser_scan->header.frame_id);
    tf2::impl::Converter<false, true>::convert(q, min_q.quaternion);

    q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
    inc_q.header = min_q.header;
    tf2::impl::Converter<false, true>::convert(q, inc_q.quaternion);
    try {
      tf_->transform(min_q, min_q, base_frame_id_, TRANSFORM_TIMEOUT);
      tf_->transform(inc_q, inc_q, base_frame_id_, TRANSFORM_TIMEOUT);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "Unable to transform min/max laser angles into base frame: %s",
        e.what());
      return;
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
    assert(ldata.ranges);
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

    lasers_[laser_index]->UpdateSensor(pf_, reinterpret_cast<SensorData *>(&ldata));

    lasers_update_[laser_index] = false;

    pf_odom_pose_ = pose;

    // Resample the particles
    if (!(++resample_count_ % resample_interval_)) {
      pf_update_resample(pf_);
      resampled = true;
    }

    pf_sample_set_t * set = pf_->sets + pf_->current_set;
    RCLCPP_DEBUG(get_logger(), "Num samples: %d\n", set->sample_count);

    // Publish the resulting cloud
    // TODO(?): set maximum rate for publishing
    if (!m_force_update) {
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
  }

  if (resampled || force_publication) {
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for (int hyp_count = 0;
      hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov)) {
        RCLCPP_ERROR(get_logger(), "Couldn't get stats on cluster %d", hyp_count);
        break;
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

      /*
         puts("");
         pf_matrix_fprintf(hyps[max_weight_hyp].pf_pose_cov, stdout, "%6.3f");
         puts("");
       */

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
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      // p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
      p.pose.covariance[6 * 5 + 5] = set->cov.m[2][2];

      /*
         printf("cov:\n");
         for(int i=0; i<6; i++)
         {
         for(int j=0; j<6; j++)
         printf("%6.3f ", p.covariance[6*i+j]);
         puts("");
         }
       */

      RCLCPP_INFO(get_logger(), "AmclNode publishing pose");
      pose_pub_->publish(p);
      last_published_pose = p;

      RCLCPP_DEBUG(get_logger(), "New pose: %6.3f %6.3f %6.3f",
        hyps[max_weight_hyp].pf_pose_mean.v[0],
        hyps[max_weight_hyp].pf_pose_mean.v[1],
        hyps[max_weight_hyp].pf_pose_mean.v[2]);

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

        this->tf_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_, TRANSFORM_TIMEOUT);
      } catch (tf2::TransformException) {
        RCLCPP_DEBUG(get_logger(), "Failed to subtract base to odom transform");
        return;
      }

      tf2::impl::Converter<true, false>::convert(odom_to_map.pose, latest_tf_);
      latest_tf_valid_ = true;

      if (tf_broadcast_ == true) {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        auto stamp = tf2_ros::fromMsg(laser_scan->header.stamp);
        tf2::TimePoint transform_expiration = stamp + transform_tolerance_;
        geometry_msgs::msg::TransformStamped tmp_tf_stamped;
        tmp_tf_stamped.header.frame_id = global_frame_id_;
        tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
        tmp_tf_stamped.child_frame_id = odom_frame_id_;
        tf2::impl::Converter<false, true>::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
        this->tfb_->sendTransform(tmp_tf_stamped);
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
      geometry_msgs::msg::TransformStamped tmp_tf_stamped;
      tmp_tf_stamped.header.frame_id = global_frame_id_;
      tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
      tmp_tf_stamped.child_frame_id = odom_frame_id_;
      tf2::impl::Converter<false, true>::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
      this->tfb_->sendTransform(tmp_tf_stamped);
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

void
AmclNode::initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  handleInitialPoseMessage(*msg);
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
  } else if (strutils::stripLeadingSlash(msg.header.frame_id) != global_frame_id_) {
    RCLCPP_WARN(
      get_logger(), "Ignoring initial pose in frame \"%s\"; initial poses must be in the global"
      " frame, \"%s\"",
      strutils::stripLeadingSlash(msg.header.frame_id).c_str(),
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
