/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <nav2_costmap_2d/layered_costmap.h>
#include <nav2_costmap_2d/costmap_2d_ros.h>
#include <cstdio>
#include <string>
#include <sys/time.h>
#include <algorithm>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "nav2_util/duration_conversions.h"

using namespace std;

namespace nav2_costmap_2d
{

Costmap2DROS::Costmap2DROS(const std::string & name, tf2_ros::Buffer & tf)
  : Node(name),
  layered_costmap_(NULL),
  name_(name),
  tf_(tf),
  transform_tolerance_(0.3),
  map_update_thread_shutdown_(false),
  stop_updates_(false),
  initialized_(true),
  stopped_(false),
  robot_stopped_(false),
  map_update_thread_(NULL),
  last_publish_(0),
  plugin_loader_("nav2_costmap_2d", "nav2_costmap_2d::Layer"),
  publisher_(NULL),
  publish_cycle_(1),
  footprint_padding_(0.0)
{
  tf2::toMsg(tf2::Transform::getIdentity(), old_pose_.pose);

  node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

  // Set Parameters if not set 
  set_parameter_if_not_set("transform_tolerance",0.3);
  set_parameter_if_not_set("update_frequency", 5.0);
  set_parameter_if_not_set("publish_frequency", 0.0); 
  set_parameter_if_not_set("width", 10);
  set_parameter_if_not_set("height", 10);
  set_parameter_if_not_set("resolution", 0.1);
  set_parameter_if_not_set("origin_x", 0.0);
  set_parameter_if_not_set("origin_y", 0.0);
  set_parameter_if_not_set("footprint", "[]");  
  set_parameter_if_not_set("footprint_padding", 0.01);
  set_parameter_if_not_set("robot_radius", 0.1); 

  // get two frames
  parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(node_);

  get_parameter_or<std::string>("global_frame", global_frame_, std::string("map"));
  get_parameter_or<std::string>("robot_base_frame", robot_base_frame_, std::string("base_link"));

  rclcpp::Time last_error = now();
  std::string tf_error;

  // we need to make sure that the transform between the robot base frame and the global frame is available
  while (rclcpp::ok() &&
      !tf_.canTransform(global_frame_, robot_base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.1), &tf_error))
  {
    if (last_error + nav2_util::durationFromSeconds(5.0) < now()) {
      RCLCPP_WARN(get_logger(),
          "Timed out waiting for transform from %s to %s to become available before running costmap, tf error: %s",
          robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
      last_error = now();
    }
    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation.
    tf_error.clear();
  }

  // check if we want a rolling window version of the costmap
  bool rolling_window, track_unknown_space, always_send_full_costmap;
  get_parameter_or<bool>("rolling_window", rolling_window, false);
  get_parameter_or<bool>("track_unknown_space", track_unknown_space, false);
  get_parameter_or<bool>("always_send_full_costmap", always_send_full_costmap, false);

  layered_costmap_ = new LayeredCostmap(global_frame_, rolling_window, track_unknown_space);

  // add and initialize layer plugins
  if (!parameters_client_->has_parameter("plugin_names") ||
    !parameters_client_->has_parameter("plugin_types") ) {
  setPluginParams(node_);
  }

  // if (parameters_client_->has_parameter("plugin_names") &&
  //   parameters_client_->has_parameter("plugin_types")) {
  //   auto param = get_parameters({"plugin_names", "plugin_types"});
  //   for (int32_t i = 0; i < param[0].get_value<std::vector<std::string>>().size(); ++i) {
  //     std::string pname = (param[0].get_value<std::vector<std::string>>())[i];
  //     std::string type = (param[1].get_value<std::vector<std::string>>())[i];
  //     RCLCPP_INFO(get_logger(), "Using plugin \"%s\"", pname.c_str());
  //     std::shared_ptr<Layer> plugin = plugin_loader_.createSharedInstance(type);
  //     layered_costmap_->addPlugin(plugin);
  //     plugin->initialize(layered_costmap_, name + "_" + pname, &tf_, node_);
  //   }
  // }

  std::vector<std::string> plugin_names = {"static_layer","inflation_layer"};
  std::vector<std::string> plugin_types = {"nav2_costmap_2d::StaticLayer","nav2_costmap_2d::InflationLayer"};
  for (int i = 0; i < 2; i++) {
    std::shared_ptr<Layer> plugin = plugin_loader_.createSharedInstance(plugin_types[i]);
    layered_costmap_->addPlugin(plugin);
    plugin->initialize(layered_costmap_, name + "_" + plugin_names[i], &tf_, node_);
  }

  // subscribe to the footprint topic
  std::string topic_param, topic;
  get_parameter_or<std::string>("footprint_topic", topic_param, std::string("footprint_topic"));
  get_parameter_or<std::string>(topic_param, topic, std::string("footprint"));
  footprint_sub_ = create_subscription<geometry_msgs::msg::Polygon>(topic,
      std::bind(&Costmap2DROS::setUnpaddedRobotFootprintPolygon, this, std::placeholders::_1));
  get_parameter_or<std::string>("published_footprint_topic", topic_param, std::string("published_footprint"));
  get_parameter_or<std::string>(topic_param, topic, std::string("oriented_footprint"));

  footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      "footprint", rmw_qos_profile_default);

  setUnpaddedRobotFootprint(makeFootprintFromParams(node_));

  publisher_ = new Costmap2DPublisher(node_,
      layered_costmap_->getCostmap(), global_frame_, "costmap",
      always_send_full_costmap);

  // create a thread to handle updating the map
  stop_updates_ = false;
  initialized_ = true;
  stopped_ = false;

  // Create a timer to check if the robot is moving
  robot_stopped_ = false;
  timer_ = create_wall_timer(100ms, std::bind(&Costmap2DROS::movementCB, this));

  // Create Parameter Validator
  param_validator_ = new nav2_dynamic_params::DynamicParamsValidator(node_);

  // Add parameter with bound limits for validation
  param_validator_->add_param("transform_tolerance",rclcpp::ParameterType::PARAMETER_DOUBLE, {0,10},1);
  param_validator_->add_param("update_frequency",rclcpp::ParameterType::PARAMETER_DOUBLE, {0,100});
  param_validator_->add_param("publish_frequency",rclcpp::ParameterType::PARAMETER_DOUBLE, {0,100});
  param_validator_->add_param("width",rclcpp::ParameterType::PARAMETER_INTEGER, {0,0}, 1);
  param_validator_->add_param("height",rclcpp::ParameterType::PARAMETER_INTEGER, {0,0}, 1);
  param_validator_->add_param("resolution",rclcpp::ParameterType::PARAMETER_DOUBLE, {0,50});
  param_validator_->add_param("origin_x",rclcpp::ParameterType::PARAMETER_DOUBLE);
  param_validator_->add_param("origin_y",rclcpp::ParameterType::PARAMETER_DOUBLE);
  param_validator_->add_param("footprint_padding",rclcpp::ParameterType::PARAMETER_DOUBLE);
  param_validator_->add_param("robot_radius",rclcpp::ParameterType::PARAMETER_DOUBLE, {0,10});

  // Add Parameter Client
  dynamic_param_client_ = new nav2_dynamic_params::DynamicParamsClient(node_); 
  dynamic_param_client_->set_callback(std::bind(&Costmap2DROS::reconfigureCB, this, std::placeholders::_1));

  // Invoke callback
  // TODO(bpwilcox): Initialize callback for dynamic parameters
  auto set_parameters_results = set_parameters({
    rclcpp::Parameter("publish_frequency", 1.0),
  });
}

void Costmap2DROS::setUnpaddedRobotFootprintPolygon(
    const geometry_msgs::msg::Polygon::SharedPtr footprint)
{
  setUnpaddedRobotFootprint(toPointVector(footprint));
}

Costmap2DROS::~Costmap2DROS()
{
  map_update_thread_shutdown_ = true;
  if (map_update_thread_ != NULL) {
    map_update_thread_->join();
    delete map_update_thread_;
  }
  if (publisher_ != NULL) {
    delete publisher_;
  }

  delete layered_costmap_;
  delete param_validator_;
  delete dynamic_param_client_;
}

void Costmap2DROS::setPluginParams(rclcpp::Node::SharedPtr node)
{
  std::vector<rclcpp::Parameter> param;

  std::vector<std::string> plugin_names = {"static_layer","inflation_layer"};
  std::vector<std::string> plugin_types = {"nav2_costmap_2d::StaticLayer","nav2_costmap_2d::InflationLayer"};
  param = {rclcpp::Parameter("plugin_names",plugin_names),rclcpp::Parameter("plugin_types",plugin_types)};
  node->set_parameters(param);
}

void Costmap2DROS::reconfigureCB(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  RCLCPP_DEBUG(node_->get_logger(), "Costmap2DROS:: Event Callback");

  dynamic_param_client_->get_event_param(event,"transform_tolerance", transform_tolerance_);

  if (map_update_thread_ != NULL)
  {
    map_update_thread_shutdown_ = true;
    map_update_thread_->join();
    delete map_update_thread_;
  }
  map_update_thread_shutdown_ = false;
  double map_update_frequency = 1.0;
  dynamic_param_client_->get_event_param(event,"update_frequency", map_update_frequency);

  double map_publish_frequency = 5.0;
  dynamic_param_client_->get_event_param(event,"publish_frequency", map_publish_frequency);

  if (map_publish_frequency > 0)
    publish_cycle_ = nav2_util::durationFromSeconds(1 / map_publish_frequency);
  else
    publish_cycle_ = nav2_util::durationFromSeconds(-1);

  // find size parameters
  double resolution, origin_x, origin_y;
  int map_width_meters, map_height_meters;

  dynamic_param_client_->get_event_param(event,"width", map_width_meters); 
  dynamic_param_client_->get_event_param(event,"height", map_height_meters);
  dynamic_param_client_->get_event_param(event,"resolution", resolution);
  dynamic_param_client_->get_event_param(event,"origin_x", origin_x);
  dynamic_param_client_->get_event_param(event,"origin_y", origin_y); 
  
  if (!layered_costmap_->isSizeLocked())
  {
    layered_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
  }

  // If the padding has changed, call setUnpaddedRobotFootprint() to
  // re-apply the padding.
  float footprint_padding;
  dynamic_param_client_->get_event_param(event,"footprint_padding", footprint_padding);

  if (footprint_padding_ != footprint_padding)
  {  
    footprint_padding_ = footprint_padding;
    setUnpaddedRobotFootprint(unpadded_footprint_);
  }

  readFootprintFromConfig(event);

  map_update_thread_ = new std::thread(std::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));
}

void Costmap2DROS::readFootprintFromConfig(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  // Only change the footprint if footprint or robot_radius has
  // changed.  Otherwise we might overwrite a footprint sent on a
  // topic by a dynamic_reconfigure call which was setting some other
  // variable.

  std::string footprint;
  double robot_radius;
  dynamic_param_client_->get_event_param(event,"footprint", footprint); 
  dynamic_param_client_->get_event_param(event,"robot_radius", robot_radius); 

  if (!dynamic_param_client_->is_in_event(event, "footprint") ||
    !dynamic_param_client_->is_in_event(event, "robot_radius"))
    {
      return;
    }

  if (footprint != "" && footprint != "[]")
  {
    std::vector<geometry_msgs::msg::Point> new_footprint;
    if (makeFootprintFromString(footprint, new_footprint))
    {
        setUnpaddedRobotFootprint(new_footprint);
    }
    else
    {
        RCLCPP_ERROR(get_logger(),"Invalid footprint string from dynamic reconfigure");
    }
  }
  else
  {
    // robot_radius may be 0, but that must be intended at this point.
    setUnpaddedRobotFootprint(makeFootprintFromRadius(robot_radius));
  }
}

void Costmap2DROS::setUnpaddedRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points)
{
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  padFootprint(padded_footprint_, footprint_padding_);

  layered_costmap_->setFootprint(padded_footprint_);
}

void Costmap2DROS::movementCB()
{

  geometry_msgs::msg::PoseStamped new_pose;
  if (!getRobotPose(new_pose)) {
    RCLCPP_WARN(get_logger(),
      "Could not get robot pose, cancelling reconfiguration");
    robot_stopped_ = false;
  }
  // make sure that the robot is not moving
  else {
    old_pose_ = new_pose;

    robot_stopped_ = (tf2::Vector3(old_pose_.pose.position.x, old_pose_.pose.position.y,
          old_pose_.pose.position.z).distance(tf2::Vector3(new_pose.pose.position.x,
            new_pose.pose.position.y, new_pose.pose.position.z)) < 1e-3) &&
        (tf2::Quaternion(old_pose_.pose.orientation.x,
          old_pose_.pose.orientation.y,
          old_pose_.pose.orientation.z,
          old_pose_.pose.orientation.w).angle(tf2::Quaternion(new_pose.pose.orientation.x,
            new_pose.pose.orientation.y,
            new_pose.pose.orientation.z,
            new_pose.pose.orientation.w)) < 1e-3);
  }
}

void Costmap2DROS::mapUpdateLoop(double frequency)
{
  // the user might not want to run the loop every cycle
  if (frequency == 0.0) {
    return;
  }
  rclcpp::Rate r(frequency);
  while (rclcpp::ok() && !map_update_thread_shutdown_) {
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    updateMap();
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    RCLCPP_DEBUG(get_logger(), "Map update time: %.9f", t_diff);
    if (publish_cycle_.nanoseconds() > 0 && layered_costmap_->isInitialized()) {    
      unsigned int x0, y0, xn, yn;
      layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
      publisher_->updateBounds(x0, xn, y0, yn);

      rclcpp::Time now = this->now();

      if (last_publish_.nanoseconds() + publish_cycle_.nanoseconds() < now.nanoseconds()) {
      //if (last_publish_ + publish_cycle_ < now) {
        publisher_->publishCostmap();
        last_publish_ = now;
      }
    }
    r.sleep();
    // make sure to sleep for the remainder of our cycle time

    // TODO(bpwilcox): find ROS2 equivalent or port for r.cycletime()
/*     if (r.period() > tf2::durationFromSec(1 / frequency)) {    
      RCLCPP_WARN(get_logger(
          "Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds",
          frequency,
          r.period());
    } */
  }
}

void Costmap2DROS::updateMap()
{
  RCLCPP_DEBUG(get_logger(), "Updating Map...");

  if (!stop_updates_) {
    // get global pose
    geometry_msgs::msg::PoseStamped pose;
    if (getRobotPose(pose)) {
      double x = pose.pose.position.x,
        y = pose.pose.position.y,
        yaw = tf2::getYaw(pose.pose.orientation);

      layered_costmap_->updateMap(x, y, yaw);
      geometry_msgs::msg::PolygonStamped footprint;
      footprint.header.frame_id = global_frame_;
      footprint.header.stamp = now();
      transformFootprint(x, y, yaw, padded_footprint_, footprint);
      footprint_pub_->publish(footprint);

      initialized_ = true;
    }
  }
}

void Costmap2DROS::start()
{
  std::vector<std::shared_ptr<Layer> > * plugins = layered_costmap_->getPlugins();
  // check if we're stopped or just paused
  if (stopped_) {
    // if we're stopped we need to re-subscribe to topics
    for (vector<std::shared_ptr<Layer> >::iterator plugin = plugins->begin();
        plugin != plugins->end();
        ++plugin)
    {
      (*plugin)->activate();
    }
    stopped_ = false;
  }
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  rclcpp::Rate r(100.0);
  while (rclcpp::ok() && !initialized_) {
    r.sleep();
  }
}

void Costmap2DROS::stop()
{
  stop_updates_ = true;
  std::vector<std::shared_ptr<Layer> > * plugins = layered_costmap_->getPlugins();
  // unsubscribe from topics
  for (vector<std::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->deactivate();
  }
  initialized_ = false;
  stopped_ = true;
}

void Costmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void Costmap2DROS::resume()
{
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  rclcpp::Rate r(100.0);
  while (!initialized_) {
    r.sleep();
  }
}


void Costmap2DROS::resetLayers()
{
  Costmap2D * top = layered_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
  std::vector<std::shared_ptr<Layer> > * plugins = layered_costmap_->getPlugins();
  for (vector<std::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->reset();
  }
}

bool Costmap2DROS::getRobotPose(geometry_msgs::msg::PoseStamped & global_pose) const
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::msg::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);

  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = rclcpp::Time();

  rclcpp::Time current_time = node_->now();  // save time for checking tf delay later
  // get the global pose of the robot
  try {
    tf_.transform(robot_pose, global_pose, global_frame_);
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(get_logger(),
      "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(get_logger(),
      "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(),
      "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  // check global_pose timeout

  //TODO(bpwilcox): use toSec() function in more recent rclcpp branch
  if (current_time -global_pose.header.stamp > nav2_util::durationFromSeconds(transform_tolerance_))
  {
    RCLCPP_WARN(get_logger(),
      "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f, difference: %.4f",
      tf2::timeToSec(tf2_ros::fromMsg(current_time)),
      tf2::timeToSec(tf2_ros::fromMsg(global_pose.header.stamp)), transform_tolerance_,
      tf2::timeToSec(tf2_ros::fromMsg(current_time)) -
        tf2::timeToSec(tf2_ros::fromMsg(global_pose.header.stamp)));

    return false;
  }

  return true;
}

void Costmap2DROS::getOrientedFootprint(std::vector<geometry_msgs::msg::Point> & oriented_footprint)
const
{
  geometry_msgs::msg::PoseStamped global_pose;
  if (!getRobotPose(global_pose)) {
    return;
  }

  double yaw = tf2::getYaw(global_pose.pose.orientation);
  transformFootprint(global_pose.pose.position.x, global_pose.pose.position.y, yaw,
      padded_footprint_, oriented_footprint);
}

}  // namespace nav2_costmap_2d
