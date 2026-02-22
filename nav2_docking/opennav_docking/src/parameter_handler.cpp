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

#include "opennav_docking/parameter_handler.hpp"

namespace opennav_docking
{

using nav2::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  const nav2::LifecycleNode::SharedPtr & node,
  const rclcpp::Logger & logger)
: nav2_util::ParameterHandler<Parameters>(node, logger)
{
  params_.controller_frequency = node->declare_or_get_parameter("controller_frequency", 50.0);
  params_.initial_perception_timeout = node->declare_or_get_parameter("initial_perception_timeout",
    5.0);
  params_.wait_charge_timeout = node->declare_or_get_parameter("wait_charge_timeout", 5.0);
  params_.dock_approach_timeout = node->declare_or_get_parameter("dock_approach_timeout", 30.0);
  params_.rotate_to_dock_timeout = node->declare_or_get_parameter("rotate_to_dock_timeout", 10.0);
  params_.undock_linear_tolerance = node->declare_or_get_parameter("undock_linear_tolerance", 0.05);
  params_.undock_angular_tolerance = node->declare_or_get_parameter("undock_angular_tolerance",
    0.05);
  params_.max_retries = node->declare_or_get_parameter("max_retries", 3);
  params_.base_frame = node->declare_or_get_parameter("base_frame", std::string("base_link"));
  params_.fixed_frame = node->declare_or_get_parameter("fixed_frame", std::string("odom"));
  params_.dock_prestaging_tolerance = node->declare_or_get_parameter("dock_prestaging_tolerance",
    0.5);
  params_.rotation_angular_tolerance = node->declare_or_get_parameter("rotation_angular_tolerance",
    0.05);

  RCLCPP_INFO(logger_, "Controller frequency set to %.4fHz", params_.controller_frequency);

  // Check the dock_backwards deprecated parameter
  try {
    params_.dock_backwards = node->declare_or_get_parameter<bool>("dock_backwards");
    RCLCPP_WARN(
      logger_, "Parameter dock_backwards is deprecated. "
      "Please use the dock_direction parameter in your dock plugin instead.");
  } catch (...) {
  }
  params_.odom_topic = node->declare_or_get_parameter("odom_topic", std::string("odom"));
  params_.odom_duration = node->declare_or_get_parameter("odom_duration", 0.3);
}

rcl_interfaces::msg::SetParametersResult ParameterHandler::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    // If we are trying to change the parameter of a plugin we can just skip it at this point
    // as they handle parameter changes themselves and don't need to lock the mutex
    if (param_name.find('.') != std::string::npos) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "controller_frequency" && parameter.as_double() <= 0.0) {
        RCLCPP_WARN(
        logger_, "The value of parameter '%s' is incorrectly set to %f, "
        "it should be >0. Ignoring parameter update.",
        param_name.c_str(), parameter.as_double());
        result.successful = false;
      } else if (parameter.as_double() < 0.0) {
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
ParameterHandler::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find('.') != std::string::npos) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "controller_frequency") {
        params_.controller_frequency = parameter.as_double();
      } else if (param_name == "initial_perception_timeout") {
        params_.initial_perception_timeout = parameter.as_double();
      } else if (param_name == "wait_charge_timeout") {
        params_.wait_charge_timeout = parameter.as_double();
      } else if (param_name == "undock_linear_tolerance") {
        params_.undock_linear_tolerance = parameter.as_double();
      } else if (param_name == "undock_angular_tolerance") {
        params_.undock_angular_tolerance = parameter.as_double();
      } else if (param_name == "rotation_angular_tolerance") {
        params_.rotation_angular_tolerance = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_STRING) {
      if (param_name == "base_frame") {
        params_.base_frame = parameter.as_string();
      } else if (param_name == "fixed_frame") {
        params_.fixed_frame = parameter.as_string();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == "max_retries") {
        params_.max_retries = parameter.as_int();
      }
    }
  }
}

}  // namespace opennav_docking
