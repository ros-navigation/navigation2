// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#include "nav2_graceful_controller/parameter_handler.hpp"

namespace nav2_graceful_controller
{

using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  const nav2::LifecycleNode::SharedPtr & node, std::string & plugin_name,
  rclcpp::Logger & logger, const double costmap_size_x)
: nav2_util::ParameterHandler<Parameters>(node, logger)
{
  plugin_name_ = plugin_name;

  params_.transform_tolerance = node->declare_or_get_parameter(
    plugin_name_ + ".transform_tolerance", 0.1);
  params_.min_lookahead = node->declare_or_get_parameter(
    plugin_name_ + ".min_lookahead", 0.25);
  params_.max_lookahead = node->declare_or_get_parameter(
    plugin_name_ + ".max_lookahead", 1.0);
  params_.max_robot_pose_search_dist = node->declare_or_get_parameter(
    plugin_name_ + ".max_robot_pose_search_dist", costmap_size_x / 2.0);
  if (params_.max_robot_pose_search_dist < 0.0) {
    RCLCPP_WARN(
      logger_, "Max robot search distance is negative, setting to max to search"
      " every point on path for the closest value.");
    params_.max_robot_pose_search_dist = std::numeric_limits<double>::max();
  }

  params_.k_phi = node->declare_or_get_parameter(
    plugin_name_ + ".k_phi", 2.0);
  params_.k_delta = node->declare_or_get_parameter(
    plugin_name_ + ".k_delta", 1.0);
  params_.beta = node->declare_or_get_parameter(
    plugin_name_ + ".beta", 0.4);
  params_.lambda = node->declare_or_get_parameter(
    plugin_name_ + ".lambda", 2.0);
  params_.v_linear_min = node->declare_or_get_parameter(
    plugin_name_ + ".v_linear_min", 0.1);
  params_.v_linear_max = node->declare_or_get_parameter(
    plugin_name_ + ".v_linear_max", 0.5);
  params_.v_angular_max = node->declare_or_get_parameter(
    plugin_name_ + ".v_angular_max", 1.0);
  params_.v_angular_min_in_place = node->declare_or_get_parameter(
    plugin_name_ + ".v_angular_min_in_place", 0.25);
  params_.slowdown_radius = node->declare_or_get_parameter(
    plugin_name_ + ".slowdown_radius", 1.5);
  params_.initial_rotation = node->declare_or_get_parameter(
    plugin_name_ + ".initial_rotation", true);
  params_.initial_rotation_tolerance = node->declare_or_get_parameter(
    plugin_name_ + ".initial_rotation_tolerance", 0.75);
  params_.prefer_final_rotation = node->declare_or_get_parameter(
    plugin_name_ + ".prefer_final_rotation", true);
  params_.rotation_scaling_factor = node->declare_or_get_parameter(
    plugin_name_ + ".rotation_scaling_factor", 0.5);
  params_.allow_backward = node->declare_or_get_parameter(
    plugin_name_ + ".allow_backward", false);
  params_.in_place_collision_resolution = node->declare_or_get_parameter(
    plugin_name_ + ".in_place_collision_resolution", 0.1);
  params_.use_collision_detection = node->declare_or_get_parameter(
    plugin_name_ + ".use_collision_detection", true);
  if (params_.initial_rotation && params_.allow_backward) {
    RCLCPP_WARN(
      logger_, "Initial rotation and allow backward parameters are both true, "
      "setting allow backward to false.");
    params_.allow_backward = false;
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
      if (parameter.as_double() < 0.0) {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %f, "
          "it should be >=0. Ignoring parameter update.",
          param_name.c_str(), parameter.as_double());
        result.successful = false;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == plugin_name_ + ".allow_backward") {
        if (params_.initial_rotation && parameter.as_bool()) {
          RCLCPP_WARN(
            logger_, "Initial rotation and allow backward parameters are both true, "
            "rejecting parameter change.");
          result.successful = false;
        }
      } else if (param_name == plugin_name_ + ".initial_rotation") {
        if (parameter.as_bool() && params_.allow_backward) {
          RCLCPP_WARN(
            logger_, "Initial rotation and allow backward parameters are both true, "
            "rejecting parameter change.");
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
      if (param_name == plugin_name_ + ".transform_tolerance") {
        params_.transform_tolerance = parameter.as_double();
      } else if (param_name == plugin_name_ + ".min_lookahead") {
        params_.min_lookahead = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_lookahead") {
        params_.max_lookahead = parameter.as_double();
      } else if (param_name == plugin_name_ + ".k_phi") {
        params_.k_phi = parameter.as_double();
      } else if (param_name == plugin_name_ + ".k_delta") {
        params_.k_delta = parameter.as_double();
      } else if (param_name == plugin_name_ + ".beta") {
        params_.beta = parameter.as_double();
      } else if (param_name == plugin_name_ + ".lambda") {
        params_.lambda = parameter.as_double();
      } else if (param_name == plugin_name_ + ".v_linear_min") {
        params_.v_linear_min = parameter.as_double();
      } else if (param_name == plugin_name_ + ".v_linear_max") {
        params_.v_linear_max = parameter.as_double();
        params_.v_linear_max_initial = params_.v_linear_max;
      } else if (param_name == plugin_name_ + ".v_angular_max") {
        params_.v_angular_max = parameter.as_double();
        params_.v_angular_max_initial = params_.v_angular_max;
      } else if (param_name == plugin_name_ + ".v_angular_min_in_place") {
        params_.v_angular_min_in_place = parameter.as_double();
      } else if (param_name == plugin_name_ + ".slowdown_radius") {
        params_.slowdown_radius = parameter.as_double();
      } else if (param_name == plugin_name_ + ".initial_rotation_tolerance") {
        params_.initial_rotation_tolerance = parameter.as_double();
      } else if (param_name == plugin_name_ + ".rotation_scaling_factor") {
        params_.rotation_scaling_factor = parameter.as_double();
      } else if (param_name == plugin_name_ + ".in_place_collision_resolution") {
        params_.in_place_collision_resolution = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == plugin_name_ + ".initial_rotation") {
        params_.initial_rotation = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".prefer_final_rotation") {
        params_.prefer_final_rotation = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".allow_backward") {
        params_.allow_backward = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".use_collision_detection") {
        params_.use_collision_detection = parameter.as_bool();
      }
    }
  }
}

}  // namespace nav2_graceful_controller
