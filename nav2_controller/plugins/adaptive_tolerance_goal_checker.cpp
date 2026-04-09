// Copyright (c) 2026, David Grbac
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
#include <cmath>

#include "nav2_controller/plugins/adaptive_tolerance_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

AdaptiveToleranceGoalChecker::AdaptiveToleranceGoalChecker()
: fine_xy_goal_tolerance_(0.10),
  fine_xy_goal_tolerance_sq_(0.01),
  coarse_xy_goal_tolerance_(0.25),
  coarse_xy_goal_tolerance_sq_(0.0625),
  yaw_goal_tolerance_(0.25),
  path_length_tolerance_(1.0),
  stateful_(true),
  symmetric_yaw_tolerance_(false),
  trans_stopped_velocity_(0.10),
  rot_stopped_velocity_(0.10),
  required_stagnation_cycles_(15),
  check_xy_(true),
  in_tolerance_zone_(false),
  no_improvement_count_(0)
{
}

AdaptiveToleranceGoalChecker::~AdaptiveToleranceGoalChecker()
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

void AdaptiveToleranceGoalChecker::initialize(
  const nav2::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  node_ = parent;
  auto node = node_.lock();
  logger_ = node->get_logger();

  fine_xy_goal_tolerance_ = node->declare_or_get_parameter(
    plugin_name + ".fine_xy_goal_tolerance", 0.10);
  coarse_xy_goal_tolerance_ = node->declare_or_get_parameter(
    plugin_name + ".coarse_xy_goal_tolerance", 0.25);
  yaw_goal_tolerance_ = node->declare_or_get_parameter(
    plugin_name + ".yaw_goal_tolerance", 0.25);
  path_length_tolerance_ = node->declare_or_get_parameter(
    plugin_name + ".path_length_tolerance", 1.0);
  stateful_ = node->declare_or_get_parameter(plugin_name + ".stateful", true);
  symmetric_yaw_tolerance_ = node->declare_or_get_parameter(
    plugin_name + ".symmetric_yaw_tolerance", false);
  trans_stopped_velocity_ = node->declare_or_get_parameter(
    plugin_name + ".trans_stopped_velocity", 0.10);
  rot_stopped_velocity_ = node->declare_or_get_parameter(
    plugin_name + ".rot_stopped_velocity", 0.10);
  required_stagnation_cycles_ = node->declare_or_get_parameter(
    plugin_name + ".required_stagnation_cycles", 15);

  fine_xy_goal_tolerance_sq_ = fine_xy_goal_tolerance_ * fine_xy_goal_tolerance_;
  coarse_xy_goal_tolerance_sq_ = coarse_xy_goal_tolerance_ * coarse_xy_goal_tolerance_;

  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &AdaptiveToleranceGoalChecker::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &AdaptiveToleranceGoalChecker::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
}

void AdaptiveToleranceGoalChecker::reset()
{
  check_xy_ = true;
  in_tolerance_zone_ = false;
  no_improvement_count_ = 0;
}

bool AdaptiveToleranceGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity,
  const nav_msgs::msg::Path & transformed_global_plan)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  // Skip check if local plan is still long (robot is far from goal region)
  if (nav2_util::geometry_utils::calculate_path_length(transformed_global_plan) >
    path_length_tolerance_)
  {
    return false;
  }

  if (check_xy_) {
    const double dx = query_pose.position.x - goal_pose.position.x;
    const double dy = query_pose.position.y - goal_pose.position.y;
    const double dist_sq = dx * dx + dy * dy;

    // Tier 1: Tight (desired) tolerance — immediate acceptance
    if (dist_sq <= fine_xy_goal_tolerance_sq_) {
      if (stateful_) {
        check_xy_ = false;
      }
      RCLCPP_INFO(
        logger_,
        "AdaptiveToleranceGoalChecker: accepting goal at fine tolerance "
        "(current: %.3f m, tol: %.3f m)",
        std::sqrt(dist_sq), fine_xy_goal_tolerance_);
      // Fall through to yaw check

    // Tier 2: Within the coarse tolerance zone — check velocity stagnation
    } else if (dist_sq <= coarse_xy_goal_tolerance_sq_) {
      // Just entered the zone: initialize tracking
      if (!in_tolerance_zone_) {
        in_tolerance_zone_ = true;
        no_improvement_count_ = 0;
        return false;
      }

      // Check if the robot is effectively stopped
      const bool robot_stopped =
        std::hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_ &&
        std::fabs(velocity.angular.z) <= rot_stopped_velocity_;

      if (robot_stopped) {
        no_improvement_count_++;
      } else {
        no_improvement_count_ = 0;
      }

      if (no_improvement_count_ < required_stagnation_cycles_) {
        return false;
      }

      // Stagnated for enough cycles: accept at coarse tolerance
      RCLCPP_INFO(
        logger_,
        "AdaptiveToleranceGoalChecker: accepting goal at coarse tolerance "
        "(current: %.3f m, fine tol: %.3f m, coarse tol: %.3f m)",
        std::sqrt(dist_sq), fine_xy_goal_tolerance_, coarse_xy_goal_tolerance_);

      if (stateful_) {
        check_xy_ = false;
      }
      // Fall through to yaw check
    } else {
      // Outside both tolerances: reset tracking state
      in_tolerance_zone_ = false;
      no_improvement_count_ = 0;
      return false;
    }
  }

  // XY is satisfied — check yaw
  const double query_yaw = tf2::getYaw(query_pose.orientation);
  const double goal_yaw = tf2::getYaw(goal_pose.orientation);

  if (symmetric_yaw_tolerance_) {
    const double dyaw_forward = angles::shortest_angular_distance(query_yaw, goal_yaw);
    const double dyaw_backward = angles::shortest_angular_distance(
      query_yaw, angles::normalize_angle(goal_yaw + M_PI));
    return std::fabs(dyaw_forward) <= yaw_goal_tolerance_ ||
           std::fabs(dyaw_backward) <= yaw_goal_tolerance_;
  }

  const double dyaw = angles::shortest_angular_distance(query_yaw, goal_yaw);
  return std::fabs(dyaw) <= yaw_goal_tolerance_;
}

bool AdaptiveToleranceGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  const double invalid_field = std::numeric_limits<double>::lowest();

  // Report max tolerance as the worst-case bound
  pose_tolerance.position.x = coarse_xy_goal_tolerance_;
  pose_tolerance.position.y = coarse_xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = trans_stopped_velocity_;
  vel_tolerance.linear.y = trans_stopped_velocity_;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = rot_stopped_velocity_;

  return true;
}

rcl_interfaces::msg::SetParametersResult
AdaptiveToleranceGoalChecker::validateParameterUpdatesCallback(
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
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == plugin_name_ + ".required_stagnation_cycles" &&
        parameter.as_int() < 1)
      {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %ld, "
          "it should be >= 1. Ignoring parameter update.",
          param_name.c_str(), parameter.as_int());
        result.successful = false;
      }
    }
  }
  return result;
}

void
AdaptiveToleranceGoalChecker::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".fine_xy_goal_tolerance") {
        fine_xy_goal_tolerance_ = parameter.as_double();
        fine_xy_goal_tolerance_sq_ = fine_xy_goal_tolerance_ * fine_xy_goal_tolerance_;
      } else if (param_name == plugin_name_ + ".coarse_xy_goal_tolerance") {
        coarse_xy_goal_tolerance_ = parameter.as_double();
        coarse_xy_goal_tolerance_sq_ = coarse_xy_goal_tolerance_ * coarse_xy_goal_tolerance_;
      } else if (param_name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".path_length_tolerance") {
        path_length_tolerance_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".trans_stopped_velocity") {
        trans_stopped_velocity_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".rot_stopped_velocity") {
        rot_stopped_velocity_ = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".symmetric_yaw_tolerance") {
        symmetric_yaw_tolerance_ = parameter.as_bool();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == plugin_name_ + ".required_stagnation_cycles") {
        required_stagnation_cycles_ = static_cast<int>(parameter.as_int());
      }
    }
  }
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::AdaptiveToleranceGoalChecker, nav2_core::GoalChecker)
