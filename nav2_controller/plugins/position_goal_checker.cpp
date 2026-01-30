// Copyright (c) 2025 Prabhav Saxena
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
#include "nav2_controller/plugins/position_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

PositionGoalChecker::PositionGoalChecker()
: xy_goal_tolerance_(0.25),
  xy_goal_tolerance_sq_(0.0625),
  path_length_tolerance_(1.0),
  stateful_(true),
  position_reached_(false)
{
}

PositionGoalChecker::~PositionGoalChecker()
{
  auto node = node_.lock();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();
}

void PositionGoalChecker::initialize(
  const nav2::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  node_ = parent;
  auto node = node_.lock();
  logger_ = node->get_logger();

  xy_goal_tolerance_ = node->declare_or_get_parameter(plugin_name + ".xy_goal_tolerance", 0.25);
  path_length_tolerance_ = node->declare_or_get_parameter(
    plugin_name + ".path_length_tolerance", 1.0);
  stateful_ = node->declare_or_get_parameter(plugin_name + ".stateful", true);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;

   // Add callback for dynamic parameters
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &PositionGoalChecker::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &PositionGoalChecker::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
}

void PositionGoalChecker::reset()
{
  position_reached_ = false;
}

bool PositionGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &, const nav_msgs::msg::Path & transformed_global_plan)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  // If the local plan length is longer than the tolerance, we skip the check
  if (nav2_util::geometry_utils::calculate_path_length(transformed_global_plan) >
    path_length_tolerance_)
  {
    return false;
  }
  // If stateful and position was already reached, maintain state
  if (stateful_ && position_reached_) {
    return true;
  }

  // Check if position is within tolerance
  double dx = query_pose.position.x - goal_pose.position.x;
  double dy = query_pose.position.y - goal_pose.position.y;

  bool position_reached = (dx * dx + dy * dy <= xy_goal_tolerance_sq_);

  // If stateful, remember that we reached the position
  if (stateful_ && position_reached) {
    position_reached_ = true;
  }

  return position_reached;
}

bool PositionGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;

  // Return zero orientation tolerance as we don't check it
  pose_tolerance.orientation.x = 0.0;
  pose_tolerance.orientation.y = 0.0;
  pose_tolerance.orientation.z = 0.0;
  pose_tolerance.orientation.w = 1.0;

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

void nav2_controller::PositionGoalChecker::setXYGoalTolerance(double tolerance)
{
  xy_goal_tolerance_ = tolerance;
  xy_goal_tolerance_sq_ = tolerance * tolerance;
}

rcl_interfaces::msg::SetParametersResult
PositionGoalChecker::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (parameter.as_double() < 0.0) {
        RCLCPP_WARN(
        logger_, "The value of parameter '%s' is incorrectly set to %f, "
        "it should be >=0. Ignoring parameter update.",
        param_name.c_str(), parameter.as_double());
        result.successful = false;
      }
    }
  }
  return result;
}

void
PositionGoalChecker::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".xy_goal_tolerance") {
        xy_goal_tolerance_ = parameter.as_double();
        xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
      } else if (param_name == plugin_name_ + ".path_length_tolerance") {
        path_length_tolerance_ = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      }
    }
  }
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::PositionGoalChecker, nav2_core::GoalChecker)
