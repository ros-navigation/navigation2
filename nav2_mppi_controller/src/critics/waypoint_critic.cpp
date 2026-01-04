// Copyright (c) 2025
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

#include "nav2_mppi_controller/critics/waypoint_critic.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>

namespace mppi::critics
{

void WaypointCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  auto getParam = parameters_handler_->getParamGetter(name_);
  
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 1.0f);
  getParam(position_tolerance_, "position_tolerance", 0.1f);
  getParam(yaw_tolerance_, "yaw_tolerance", 0.1f);
  getParam(waypoint_topic_, "waypoint_topic", std::string("truncated_path"));
  getParam(use_orientation_, "use_orientation", true);
  getParam(max_waypoint_distance_, "max_waypoint_distance", 10.0f);

  RCLCPP_INFO(
    logger_, 
    "WaypointCritic instantiated with %d power, %f weight, "
    "position tolerance: %f, yaw tolerance: %f, topic: %s",
    power_, weight_, position_tolerance_, yaw_tolerance_, waypoint_topic_.c_str());

  // Create subscription to waypoint topic
  auto node = parent_.lock();
  if (node) {
    waypoint_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      waypoint_topic_,
      std::bind(&WaypointCritic::waypointCallback, this, std::placeholders::_1),
      rclcpp::QoS(10).reliable());
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node, false);

    RCLCPP_INFO(logger_, "WaypointCritic subscribed to topic: %s", waypoint_topic_.c_str());
  } else {
    RCLCPP_ERROR(logger_, "Failed to get parent node for WaypointCritic subscription");
  }
}

void WaypointCritic::waypointCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(waypoints_mutex_);
  waypoints_ = msg->poses;
  waypoints_frame_ = msg->header.frame_id;
  RCLCPP_INFO(
    logger_, 
    "WaypointCritic received %zu waypoints", 
    waypoints_.size());
}

void WaypointCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(waypoints_mutex_);
  
  // If no waypoints available, don't apply any cost
  if (waypoints_.empty()) {
    return;
  }

  // Get current robot pose
  const auto & robot_x = data.state.pose.pose.position.x;
  const auto & robot_y = data.state.pose.pose.position.y;
  
  // Always target the first waypoint (assuming completed waypoints are removed by publisher)
  int target_waypoint_idx = getTargetWaypoint();
  
  if (target_waypoint_idx < 0) {
    return;
  }

  const std::string target_frame = data.state.pose.header.frame_id;
  geometry_msgs::msg::PoseStamped wp_in = waypoints_[target_waypoint_idx];
  if (wp_in.header.frame_id.empty()) {
    wp_in.header.frame_id = waypoints_frame_;
  }
  geometry_msgs::msg::PoseStamped wp_tf;
  if (wp_in.header.frame_id == target_frame || !tf_buffer_) {
    wp_tf = wp_in;
  } else {
    if (!tf_buffer_->canTransform(
        target_frame, wp_in.header.frame_id, wp_in.header.stamp,
        tf2::durationFromSec(0.0)))
    {
    RCLCPP_WARN(logger_, "TF %s->%s not available yet", 
                wp_in.header.frame_id.c_str(), target_frame.c_str());
    return;
    }
    tf_buffer_->transform(wp_in, wp_tf, target_frame, tf2::durationFromSec(0.0));
  }
  const float target_x = static_cast<float>(wp_tf.pose.position.x);
  const float target_y = static_cast<float>(wp_tf.pose.position.y);
  const float target_yaw = static_cast<float>(tf2::getYaw(wp_tf.pose.orientation));


//   const auto & plan = data.path;
//   if (plan.x.size() == 0) {
//     return;
//   }
//   RCLCPP_INFO(
//   logger_,
//   "data.path: size=%zu frame=%s",
//   static_cast<size_t>(plan.x.size()),
//   data.state.pose.header.frame_id.c_str());
//   const float target_x   = plan.x(0);
//   const float target_y   = plan.y(0);
//   const float target_yaw = (plan.yaws.size() > 0) ? plan.yaws(0) : 0.0f;


  // Check if waypoint is within reasonable distance
  double waypoint_distance = calculateDistance(robot_x, robot_y, target_x, target_y);
  if (waypoint_distance > max_waypoint_distance_) {
    return;
  }


  // Calculate position costs for all trajectories
  const auto delta_x = data.trajectories.x - target_x;
  const auto delta_y = data.trajectories.y - target_y;
    // RCLCPP_INFO(
    // logger_,
    // "Traj mean: x=%.3f y=%.3f | target=(%.3f, %.3f)",
    // static_cast<double>(data.trajectories.x.mean()),
    // static_cast<double>(data.trajectories.y.mean()),
    // target_x, target_y);
  
  // Calculate euclidean distance cost
  auto position_distances = (delta_x.square() + delta_y.square()).sqrt();
  
  // Apply position tolerance - penalize trajectories that are farther than tolerance
  auto position_costs = (position_distances - position_tolerance_).cwiseMax(0.0f);

  // If orientation is considered, add yaw cost
  Eigen::ArrayXf total_costs = position_costs;
  
  if (use_orientation_) {
    // Calculate yaw differences for all trajectories using utils function  
    auto yaw_differences = utils::shortest_angular_distance(data.trajectories.yaws, target_yaw);
    
    // Apply yaw tolerance
    auto yaw_costs = (yaw_differences.abs() - yaw_tolerance_).cwiseMax(0.0f);
    
    // Combine position and yaw costs
    total_costs = position_costs + yaw_costs;
  }

  // Apply power and weight
  if (power_ > 1u) {
    data.costs += (total_costs.rowwise().mean() * weight_).pow(power_);
  } else {
    data.costs += (total_costs.rowwise().mean() * weight_).eval();
  }
}

int WaypointCritic::getTargetWaypoint()
{
  if (waypoints_.empty()) {
    return -1;
  }

  // Always target the first waypoint in the list
  // (assuming the publisher removes completed waypoints)
  return 0;
}

double WaypointCritic::calculateDistance(double x1, double y1, double x2, double y2)
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  return sqrt(dx * dx + dy * dy);
}

double WaypointCritic::calculateAngularDifference(double angle1, double angle2)
{
  double diff = angle2 - angle1;
  while (diff > M_PI) diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;
  return diff;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::WaypointCritic, mppi::critics::CriticFunction)