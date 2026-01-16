// Copyright (c) 2022 Samsung Research America
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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav2_regulated_pure_pursuit_controller/parameter_handler.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  const nav2::LifecycleNode::SharedPtr & node,
  std::string & plugin_name, rclcpp::Logger & logger,
  const double costmap_size_x)
: nav2_util::ParameterHandler<Parameters>(node, logger)
{
  plugin_name_ = plugin_name;

  const std::string old_name = plugin_name_ + ".desired_linear_vel";
  const std::string new_name = plugin_name_ + ".max_linear_vel";
  try {
    params_.max_linear_vel = node->declare_or_get_parameter<double>(old_name);
    RCLCPP_WARN(
      logger_,
      "Parameter '%s' is deprecated. Use '%s' instead.",
      old_name.c_str(), new_name.c_str());
  } catch (const std::exception &) {
    params_.max_linear_vel = node->declare_or_get_parameter(new_name, 0.5);
  }
  params_.base_max_linear_vel = params_.max_linear_vel;

  params_.min_linear_vel =
    node->declare_or_get_parameter(plugin_name_ + ".min_linear_vel", -0.5);
  params_.max_angular_vel =
    node->declare_or_get_parameter(plugin_name_ + ".max_angular_vel", 2.5);
  params_.min_angular_vel =
    node->declare_or_get_parameter(plugin_name_ + ".min_angular_vel", -2.5);
  params_.lookahead_dist =
    node->declare_or_get_parameter(plugin_name_ + ".lookahead_dist", 0.6);
  params_.min_lookahead_dist =
    node->declare_or_get_parameter(plugin_name_ + ".min_lookahead_dist", 0.3);
  params_.max_lookahead_dist =
    node->declare_or_get_parameter(plugin_name_ + ".max_lookahead_dist", 0.9);
  params_.lookahead_time =
    node->declare_or_get_parameter(plugin_name_ + ".lookahead_time", 1.5);
  params_.rotate_to_heading_angular_vel = node->declare_or_get_parameter(
    plugin_name_ + ".rotate_to_heading_angular_vel", 1.8);
  params_.use_velocity_scaled_lookahead_dist = node->declare_or_get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist", false);
  params_.min_approach_linear_velocity = node->declare_or_get_parameter(
    plugin_name_ + ".min_approach_linear_velocity", 0.05);
  params_.approach_velocity_scaling_dist = node->declare_or_get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist", 0.6);
  if (params_.approach_velocity_scaling_dist > costmap_size_x / 2.0) {
    RCLCPP_WARN(
      logger_, "approach_velocity_scaling_dist is larger than forward costmap extent, "
      "leading to permanent slowdown");
  }
  params_.max_allowed_time_to_collision_up_to_carrot =
    node->declare_or_get_parameter(
      plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot", 1.0);
  params_.min_distance_to_obstacle = node->declare_or_get_parameter(
    plugin_name_ + ".min_distance_to_obstacle", -1.0);
  params_.use_regulated_linear_velocity_scaling =
    node->declare_or_get_parameter(
      plugin_name_ + ".use_regulated_linear_velocity_scaling", true);
  params_.use_cost_regulated_linear_velocity_scaling =
    node->declare_or_get_parameter(
      plugin_name_ + ".use_cost_regulated_linear_velocity_scaling", true);
  params_.cost_scaling_dist =
    node->declare_or_get_parameter(plugin_name_ + ".cost_scaling_dist", 0.6);
  params_.cost_scaling_gain =
    node->declare_or_get_parameter(plugin_name_ + ".cost_scaling_gain", 1.0);
  params_.inflation_cost_scaling_factor = node->declare_or_get_parameter(
    plugin_name_ + ".inflation_cost_scaling_factor", 3.0);
  params_.regulated_linear_scaling_min_radius = node->declare_or_get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_radius", 0.90);
  params_.regulated_linear_scaling_min_speed = node->declare_or_get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_speed", 0.25);
  params_.use_fixed_curvature_lookahead = node->declare_or_get_parameter(
    plugin_name_ + ".use_fixed_curvature_lookahead", false);
  params_.curvature_lookahead_dist = node->declare_or_get_parameter(
    plugin_name_ + ".curvature_lookahead_dist", 0.6);
  params_.use_rotate_to_heading = node->declare_or_get_parameter(
    plugin_name_ + ".use_rotate_to_heading", true);
  params_.rotate_to_heading_min_angle = node->declare_or_get_parameter(
    plugin_name_ + ".rotate_to_heading_min_angle", 0.785);
  params_.max_linear_accel =
    node->declare_or_get_parameter(plugin_name_ + ".max_linear_accel", 2.5);
  params_.max_linear_decel =
    node->declare_or_get_parameter(plugin_name_ + ".max_linear_decel", -2.5);
  params_.max_angular_accel =
    node->declare_or_get_parameter(plugin_name_ + ".max_angular_accel", 3.2);
  params_.max_angular_decel =
    node->declare_or_get_parameter(plugin_name_ + ".max_angular_decel", -3.2);
  params_.use_cancel_deceleration = node->declare_or_get_parameter(
    plugin_name_ + ".use_cancel_deceleration", false);
  params_.cancel_deceleration = node->declare_or_get_parameter(
    plugin_name_ + ".cancel_deceleration", 3.2);
  params_.allow_reversing =
    node->declare_or_get_parameter(plugin_name_ + ".allow_reversing", false);

  params_.interpolate_curvature_after_goal = node->declare_or_get_parameter(
    plugin_name_ + ".interpolate_curvature_after_goal", false);
  if (!params_.use_fixed_curvature_lookahead && params_.interpolate_curvature_after_goal) {
    RCLCPP_WARN(
      logger_, "For interpolate_curvature_after_goal to be set to true, "
      "use_fixed_curvature_lookahead should be true, it is currently set to false. Disabling.");
    params_.interpolate_curvature_after_goal = false;
  }
  params_.use_collision_detection = node->declare_or_get_parameter(
    plugin_name_ + ".use_collision_detection", true);
  params_.stateful =
    node->declare_or_get_parameter(plugin_name_ + ".stateful", true);
  params_.use_dynamic_window =
    node->declare_or_get_parameter(plugin_name_ + ".use_dynamic_window", false);

  if (params_.inflation_cost_scaling_factor <= 0.0) {
    RCLCPP_WARN(
      logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
      "it should be >0. Disabling cost regulated linear velocity scaling.");
    params_.use_cost_regulated_linear_velocity_scaling = false;
  }
}

rcl_interfaces::msg::SetParametersResult ParameterHandler::validateParameterUpdatesCallback(
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
      const bool allow_negative =
        param_name == plugin_name_ + ".min_linear_vel" ||
        param_name == plugin_name_ + ".min_angular_vel" ||
        param_name == plugin_name_ + ".max_linear_decel" ||
        param_name == plugin_name_ + ".max_angular_decel";
      if (param_name == plugin_name_ + ".inflation_cost_scaling_factor" &&
        parameter.as_double() <= 0.0)
      {
        RCLCPP_WARN(
          logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
          "it should be >0. Ignoring parameter update.");
        result.successful = false;
      } else if (parameter.as_double() < 0.0 && !allow_negative) {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %f, "
          "it should be >=0. Ignoring parameter update.",
          param_name.c_str(), parameter.as_double());
        result.successful = false;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == plugin_name_ + ".allow_reversing") {
        if (params_.use_rotate_to_heading && parameter.as_bool()) {
          RCLCPP_WARN(
            logger_, "Both use_rotate_to_heading and allow_reversing "
            "parameter cannot be set to true. Rejecting parameter update.");
          result.successful = false;
        }
      }
    }
  }
  return result;
}

void
ParameterHandler::updateParametersCallback(
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
      if (param_name == plugin_name_ + ".inflation_cost_scaling_factor") {
        params_.inflation_cost_scaling_factor = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_linear_vel") {
        params_.max_linear_vel = parameter.as_double();
        params_.base_max_linear_vel = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_angular_accel") {
        params_.max_angular_accel = parameter.as_double();
      } else if (param_name == plugin_name_ + ".min_linear_vel") {
        params_.min_linear_vel = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_angular_vel") {
        params_.max_angular_vel = parameter.as_double();
      } else if (param_name == plugin_name_ + ".min_angular_vel") {
        params_.min_angular_vel = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_linear_accel") {
        params_.max_linear_accel = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_linear_decel") {
        params_.max_linear_decel = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_angular_decel") {
        params_.max_angular_decel = parameter.as_double();
      } else if (param_name == plugin_name_ + ".lookahead_dist") {
        params_.lookahead_dist = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_lookahead_dist") {
        params_.max_lookahead_dist = parameter.as_double();
      } else if (param_name == plugin_name_ + ".min_lookahead_dist") {
        params_.min_lookahead_dist = parameter.as_double();
      } else if (param_name == plugin_name_ + ".lookahead_time") {
        params_.lookahead_time = parameter.as_double();
      } else if (param_name == plugin_name_ + ".rotate_to_heading_angular_vel") {
        params_.rotate_to_heading_angular_vel = parameter.as_double();
      } else if (param_name == plugin_name_ + ".min_approach_linear_velocity") {
        params_.min_approach_linear_velocity = parameter.as_double();
      } else if (param_name == plugin_name_ + ".curvature_lookahead_dist") {
        params_.curvature_lookahead_dist = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot") {
        params_.max_allowed_time_to_collision_up_to_carrot = parameter.as_double();
      } else if (param_name == plugin_name_ + ".min_distance_to_obstacle") {
        params_.min_distance_to_obstacle = parameter.as_double();
      } else if (param_name == plugin_name_ + ".cost_scaling_dist") {
        params_.cost_scaling_dist = parameter.as_double();
      } else if (param_name == plugin_name_ + ".cost_scaling_gain") {
        params_.cost_scaling_gain = parameter.as_double();
      } else if (param_name == plugin_name_ + ".regulated_linear_scaling_min_radius") {
        params_.regulated_linear_scaling_min_radius = parameter.as_double();
      } else if (param_name == plugin_name_ + ".regulated_linear_scaling_min_speed") {
        params_.regulated_linear_scaling_min_speed = parameter.as_double();
      } else if (param_name == plugin_name_ + ".cancel_deceleration") {
        params_.cancel_deceleration = parameter.as_double();
      } else if (param_name == plugin_name_ + ".rotate_to_heading_min_angle") {
        params_.rotate_to_heading_min_angle = parameter.as_double();
      } else if (param_name == plugin_name_ + ".approach_velocity_scaling_dist") {
        params_.approach_velocity_scaling_dist = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        params_.use_velocity_scaled_lookahead_dist = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".use_regulated_linear_velocity_scaling") {
        params_.use_regulated_linear_velocity_scaling = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".use_fixed_curvature_lookahead") {
        params_.use_fixed_curvature_lookahead = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".use_cost_regulated_linear_velocity_scaling") {
        params_.use_cost_regulated_linear_velocity_scaling = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".use_collision_detection") {
        params_.use_collision_detection = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".stateful") {
        params_.stateful = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".use_rotate_to_heading") {
        params_.use_rotate_to_heading = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".use_cancel_deceleration") {
        params_.use_cancel_deceleration = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".allow_reversing") {
        params_.allow_reversing = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".interpolate_curvature_after_goal") {
        params_.interpolate_curvature_after_goal = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".use_dynamic_window") {
        params_.use_dynamic_window = parameter.as_bool();
      }
    }
  }
}

}  // namespace nav2_regulated_pure_pursuit_controller
