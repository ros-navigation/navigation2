// Copyright (c) 2025 Dexory
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
#include <limits>
#include <vector>

#include "angles/angles.h"
#include "nav2_controller/plugins/axis_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"


using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

AxisGoalChecker::AxisGoalChecker()
: goal_tolerance_(0.25),
  path_length_tolerance_(1.0)
{
}

void AxisGoalChecker::initialize(
  const nav2::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  nav2::declare_parameter_if_not_declared(
    node,
    plugin_name + ".goal_tolerance", rclcpp::ParameterValue(0.25));
  node->get_parameter(plugin_name + ".goal_tolerance", goal_tolerance_);
  if (goal_tolerance_ <= 0.0) {
    RCLCPP_WARN(
      node->get_logger(),
      "Parameter '%s.goal_tolerance' must be positive. Resetting to default value 0.25.",
      plugin_name.c_str());
    goal_tolerance_ = 0.25;
  }

  nav2::declare_parameter_if_not_declared(
    node,
    plugin_name + ".path_length_tolerance", rclcpp::ParameterValue(1.0));
  node->get_parameter(plugin_name + ".path_length_tolerance", path_length_tolerance_);

  nav2::declare_parameter_if_not_declared(
    node,
    plugin_name + ".is_overshoot_valid", rclcpp::ParameterValue(false));
  node->get_parameter(plugin_name + ".is_overshoot_valid", is_overshoot_valid_);

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&AxisGoalChecker::dynamicParametersCallback, this, _1));
}

void AxisGoalChecker::reset()
{
}

bool AxisGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &,
  const nav_msgs::msg::Path & transformed_global_plan)
{
  // If the local plan length is longer than the tolerance, we skip the check
  if (nav2_util::geometry_utils::calculate_path_length(transformed_global_plan) >
    path_length_tolerance_)
  {
    return false;
  }

  // Extract before_goal_pose from the path (second to last pose)
  std::optional<geometry_msgs::msg::Pose> before_goal_pose;
  if (transformed_global_plan.poses.size() >= 2) {
    before_goal_pose = transformed_global_plan.poses[transformed_global_plan.poses.size() - 2].pose;
  }

  if (before_goal_pose.has_value()) {
    // end of path direction
    double end_of_path_yaw = atan2(
      goal_pose.position.y - before_goal_pose->position.y,
      goal_pose.position.x - before_goal_pose->position.x);

    double robot_to_goal_yaw = atan2(
      goal_pose.position.y - query_pose.position.y,
      goal_pose.position.x - query_pose.position.x);

    double projection_angle = angles::shortest_angular_distance(
      robot_to_goal_yaw, end_of_path_yaw);

    double projected_distance_to_goal = std::hypot(
      goal_pose.position.x - query_pose.position.x,
      goal_pose.position.y - query_pose.position.y) *
      cos(projection_angle);

    if (is_overshoot_valid_) {
      return projected_distance_to_goal < goal_tolerance_;
    } else {
      return fabs(projected_distance_to_goal) < goal_tolerance_;
    }
  } else {
    // handle path with only 1 point, in that case reverting to single distance check
    double distance_to_goal = std::hypot(
      goal_pose.position.x - query_pose.position.x,
      goal_pose.position.y - query_pose.position.y);
    return fabs(distance_to_goal) < goal_tolerance_;
  }
}

bool AxisGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = goal_tolerance_;
  pose_tolerance.position.y = goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(M_PI_2);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

rcl_interfaces::msg::SetParametersResult
AxisGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".segment_axis_goal_tolerance") {
        goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".path_length_tolerance") {
        path_length_tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".is_overshoot_valid") {
        is_overshoot_valid_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::AxisGoalChecker, nav2_core::GoalChecker)
