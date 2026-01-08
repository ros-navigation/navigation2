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
: along_path_tolerance_(0.25), cross_track_tolerance_(0.25),
  path_length_tolerance_(1.0), is_overshoot_valid_(false)
{
}

void AxisGoalChecker::initialize(
  const nav2::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  logger_ = node->get_logger();

  along_path_tolerance_ = node->declare_or_get_parameter(
    plugin_name + ".along_path_tolerance", 0.25);
  cross_track_tolerance_ = node->declare_or_get_parameter(
    plugin_name + ".cross_track_tolerance", 0.25);
  path_length_tolerance_ = node->declare_or_get_parameter(
    plugin_name + ".path_length_tolerance", 1.0);
  is_overshoot_valid_ = node->declare_or_get_parameter(
    plugin_name + ".is_overshoot_valid", false);

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

  // Check if we have at least 2 poses to determine path direction
  if (transformed_global_plan.poses.size() >= 2) {
    // Use axis-aligned goal checking with path direction
    // Find a pose before goal that is sufficiently far from goal
    const geometry_msgs::msg::Pose * before_goal_pose_ptr = nullptr;
    double dx = 0.0;
    double dy = 0.0;

    for (int i = transformed_global_plan.poses.size() - 2; i >= 0; --i) {
      const auto & candidate_pose = transformed_global_plan.poses[i].pose;
      dx = goal_pose.position.x - candidate_pose.position.x;
      dy = goal_pose.position.y - candidate_pose.position.y;
      double pose_distance = std::hypot(dx, dy);

      if (pose_distance >= 1e-6) {
        before_goal_pose_ptr = &candidate_pose;
        break;
      }
    }

    // If all poses are too close to goal, fall back to simple distance check
    if (!before_goal_pose_ptr) {
      RCLCPP_DEBUG(
        logger_,
        "All poses in path are too close to goal, falling back to simple distance check");
      double distance_to_goal = std::hypot(
        goal_pose.position.x - query_pose.position.x,
        goal_pose.position.y - query_pose.position.y);
      double tolerance = std::hypot(along_path_tolerance_, cross_track_tolerance_);
      return distance_to_goal < tolerance;
    }

    // end of path direction
    double end_of_path_yaw = atan2(dy, dx);

    // Check if robot is already at goal (would cause atan2(0,0))
    double robot_to_goal_dx = goal_pose.position.x - query_pose.position.x;
    double robot_to_goal_dy = goal_pose.position.y - query_pose.position.y;
    double distance_to_goal = std::hypot(robot_to_goal_dx, robot_to_goal_dy);

    if (distance_to_goal < 1e-6) {
      return true;  // Robot is at goal
    }

    double robot_to_goal_yaw = atan2(robot_to_goal_dy, robot_to_goal_dx);
    double projection_angle = angles::shortest_angular_distance(
      robot_to_goal_yaw, end_of_path_yaw);
    double along_path_distance = distance_to_goal * cos(projection_angle);
    double cross_track_distance = distance_to_goal * sin(projection_angle);

    if (is_overshoot_valid_) {
      return along_path_distance < along_path_tolerance_ &&
             fabs(cross_track_distance) < cross_track_tolerance_;
    } else {
      return fabs(along_path_distance) < along_path_tolerance_ &&
             fabs(cross_track_distance) < cross_track_tolerance_;
    }
  } else {
    // Fallback: path has only 1 point, use simple distance check
    RCLCPP_DEBUG(
      logger_,
      "Path has fewer than 2 poses, falling back to simple distance check");
    double distance_to_goal = std::hypot(
      goal_pose.position.x - query_pose.position.x,
      goal_pose.position.y - query_pose.position.y);
    double tolerance = std::hypot(along_path_tolerance_, cross_track_tolerance_);
    return distance_to_goal < tolerance;
  }
}

bool AxisGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = std::min(along_path_tolerance_, cross_track_tolerance_);
  pose_tolerance.position.y = std::min(along_path_tolerance_, cross_track_tolerance_);
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
    if (name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".along_path_tolerance") {
        along_path_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".cross_track_tolerance") {
        cross_track_tolerance_ = parameter.as_double();
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
