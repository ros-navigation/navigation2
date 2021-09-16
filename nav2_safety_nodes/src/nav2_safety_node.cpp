// Copyright (c) 2021, Saurabh Suresh Powar
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <utility>

#include "nav2_safety_nodes/nav2_safety_node.hpp"

using namespace std::chrono_literals;

namespace nav2_safety_nodes
{

SafetyZone::SafetyZone()
: nav2_util::LifecycleNode("SafetyZone", "", false)
{
  logger_ = get_logger();
  RCLCPP_INFO(logger_, "Creating Safety Zone Polygon Node");

  // Vector of string for multiple LaserScan topics
  const std::vector<std::string> scan_topics{"scan"};
  // pass polygon parameters at string
  declare_parameter("safety_polygon", std::string("[]"));
  declare_parameter("update_frequency", rclcpp::ParameterValue(0.1));
  declare_parameter("zone_action", rclcpp::ParameterValue(0.0));
  declare_parameter("zone_priority", rclcpp::ParameterValue(1));
  declare_parameter("zone_num_pts", rclcpp::ParameterValue(1));
  declare_parameter("base_frame", std::string("base_link"));
  declare_parameter("tf_tolerance", rclcpp::ParameterValue(0.01));
  declare_parameter("scan_topics", scan_topics);
  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));
}

nav2_util::CallbackReturn
SafetyZone::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Configuring and Setting up");

  // Get all of the required parameters
  safety_polygon_ = get_parameter("safety_polygon").as_string();
  update_frequency_ = get_parameter("update_frequency").as_double();
  zone_action_ = get_parameter("zone_action").as_double();
  zone_priority_ = get_parameter("zone_priority").as_int();
  zone_num_pts_ = get_parameter("zone_num_pts").as_int();
  base_frame_ = get_parameter("base_frame").as_string();
  tf_tolerance_ = get_parameter("tf_tolerance").as_double();
  scan_topics_ = get_parameter("scan_topics").as_string_array();
  speed_limit_topic_ = get_parameter("speed_limit_topic").as_string();

  // If the safety_polygon has been specified, it must be in the correct format
  if (safety_polygon_ != "" && safety_polygon_ != "[]") {
    // Polygon parameter has been specified, polygon -> point vector(safety_zone)
    nav2_util::makeVectorPointsFromString(safety_polygon_, safety_zone_);
  } else {
    throw std::runtime_error("The safety_polygon is invalid");
  }

  // Initialize transform listener and broadcaster
  tf2_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf2_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SafetyZone::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating");
  // Create the publishers and subscribers
  safety_polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
    "safety_zone", rclcpp::SystemDefaultsQoS());
  // Velocity publisher
  speed_limit_pub_ = create_publisher<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic_, rclcpp::QoS(10));
  // Multiple Laserscan subscribers
  scan_subscribers_.reserve(scan_topics_.size());
  for (unsigned int i = 0; i < scan_topics_.size(); i++) {
    RCLCPP_INFO(logger_, "Subscribing to %s topic", scan_topics_[i].c_str());
    scan_subscribers_.push_back(
      create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topics_[i].c_str(), rclcpp::SystemDefaultsQoS(),
        std::bind(&SafetyZone::laserCallback, this, std::placeholders::_1)));
  }
  auto update_frequency_s_ = std::chrono::duration<double>(update_frequency_);
  timer_ = create_wall_timer(
  update_frequency_s_, std::bind(&SafetyZone::timerCallback, this));
  RCLCPP_INFO(logger_, "Subscribed to scan topics");
  safety_polygon_pub_->on_activate();
  speed_limit_pub_->on_activate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SafetyZone::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating");
  safety_polygon_pub_->on_deactivate();
  speed_limit_pub_->on_deactivate();
  timer_->cancel();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SafetyZone::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  safety_polygon_pub_.reset();
  speed_limit_pub_.reset();
  for (unsigned int i = 0; i < scan_subscribers_.size(); i++){
    scan_subscribers_[i].reset();
  }
  timer_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SafetyZone::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
SafetyZone::laserCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  // project the laser into a point cloud
  sensor_msgs::msg::PointCloud2 cloud;

  try {
    projector_.projectLaser(*message, cloud);
  } catch (...) {
    RCLCPP_WARN(logger_, "Unable to project laser into cloud");
    return;
  }

  // Transform the cloud if necessary
  if (!base_frame_.empty() && cloud.header.frame_id != base_frame_) {
    try {
      cloud = tf2_->transform(cloud, base_frame_, tf2::durationFromSec(tf_tolerance_));
      sensor_data_.push(cloud);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(logger_, "Transform failure: %s", ex.what());
      return;
    }
  } else if (cloud.header.frame_id == base_frame_) {
    sensor_data_.push(cloud);
  }
}

double
SafetyZone::dotProduct(
  const Eigen::Vector3d & pt1,
  const Eigen::Vector3d & pt2)
{
  return pt1[0] * pt2[1] - pt1[1] * pt2[0];
}

// Function for detecting if cloud points are inside safety polygon or not
int
SafetyZone::detectPoints(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::vector<geometry_msgs::msg::Point> & safety_zone)
{
  int num_points_in_zone = 0;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
  // iterating through cloud points
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    double px = *iter_x, py = *iter_y, pz = *iter_z;
    int count_same_side_results = 0, on_edge = 0;
    // iterating through polygon points
    for (int i = 0; i < int(safety_zone.size()); ++i) {
      geometry_msgs::msg::Point vertex = safety_zone[i];
      geometry_msgs::msg::Point next_vertex = safety_zone[(i + 1) % safety_zone.size()];
      Eigen::Vector3d affine_segment = {next_vertex.x - vertex.x, next_vertex.y - vertex.y, next_vertex.z - vertex.z};
      Eigen::Vector3d affine_point = {px - vertex.x, py - vertex.y, pz - vertex.z};
      double dot = dotProduct(affine_segment, affine_point);
      if (dot > 0) {
        count_same_side_results++;
      } else if (dot < 0) {
        count_same_side_results--;
      } else {
        on_edge++;
      }
    }
    // checking if point is on same side of all edges
    if (abs(count_same_side_results == int(safety_zone.size())) || on_edge == 2) {
      num_points_in_zone++;
    }
  }
  return num_points_in_zone;
}

void
SafetyZone::timerCallback()
{
  while (!sensor_data_.empty()) {
    if (detectPoints(sensor_data_.front(), safety_zone_) >= zone_num_pts_) {
      RCLCPP_INFO(logger_, "More points than %d were detected within the zone, slowing the robot by action %f", zone_num_pts_, zone_action_);
      std::unique_ptr<nav2_msgs::msg::SpeedLimit> msg =
        std::make_unique<nav2_msgs::msg::SpeedLimit>();
      msg->header.frame_id = base_frame_;
      msg->header.stamp = clock_->now();
      msg->speed_limit = zone_action_;
      speed_limit_pub_->publish(std::move(msg));
    }
    sensor_data_.pop();
  }
}
}  // namespace nav2_safety_nodes
