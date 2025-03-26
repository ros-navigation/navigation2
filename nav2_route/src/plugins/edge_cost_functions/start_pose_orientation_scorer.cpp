// Copyright (c) 2025, Polymath Robotics Inc.
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

#include <memory>
#include <string>

#include "nav2_route/plugins/edge_cost_functions/start_pose_orientation_scorer.hpp"

namespace nav2_route
{

void StartPoseOrientationScorer::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring start pose orientation scorer.");
  name_ = name;
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node,
    getName() + ".orientation_tolerance", rclcpp::ParameterValue(M_PI / 2.0));

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".orientation_weight", rclcpp::ParameterValue(1.0));

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".use_orientation_threshold", rclcpp::ParameterValue(false));

  orientation_tolerance_ = node->get_parameter(getName() + ".orientation_tolerance").as_double();
  orientation_weight_ =
    static_cast<float>(node->get_parameter(getName() + ".orientation_weight").as_double());
  route_frame_ = node->get_parameter("route_frame").as_string();
  base_frame_ = node->get_parameter("base_frame").as_string();
  use_orientation_threshold_ =
    node->get_parameter(getName() + ".use_orientation_threshold").as_bool();

  tf_buffer_ = tf_buffer;
}

bool StartPoseOrientationScorer::score(
  const EdgePtr edge,
  const geometry_msgs::msg::PoseStamped & /* start_pose */, 
  const geometry_msgs::msg::PoseStamped & /* goal_pose */, 
  const RouteData & /* route_data */,
  const EdgeType & edge_type, float & cost)
{
  if (edge_type == EdgeType::START) {
    double edge_orientation = std::atan2(
      edge->end->coords.y - edge->start->coords.y,
      edge->end->coords.x - edge->start->coords.x);
    geometry_msgs::msg::PoseStamped robot_pose = getRobotPose();
    double start_orientation = tf2::getYaw(robot_pose.pose.orientation);
    double d_yaw = std::abs(angles::shortest_angular_distance(edge_orientation, start_orientation));

    if (use_orientation_threshold_) {
      if (d_yaw > orientation_tolerance_) {
        return false;
      }
    }

    cost = orientation_weight_ * static_cast<float>(d_yaw);
  }
  return true;
}

geometry_msgs::msg::PoseStamped StartPoseOrientationScorer::getRobotPose()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!nav2_util::getCurrentPose(pose, *tf_buffer_, route_frame_, base_frame_)) {
    throw nav2_core::RouteTFError("Unable to get robot pose in route frame: " + route_frame_);
  }
  return pose;
}

std::string StartPoseOrientationScorer::getName()
{
  return name_;
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::StartPoseOrientationScorer, nav2_route::EdgeCostFunction)
