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

#include "opennav_following/parameter_handler.hpp"

namespace opennav_following
{

using nav2::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  const nav2::LifecycleNode::SharedPtr & node,
  const rclcpp::Logger & logger)
: nav2_util::ParameterHandler<Parameters>(node, logger)
{
  params_.controller_frequency = node->declare_or_get_parameter(
    "controller_frequency", 50.0);
  params_.detection_timeout = node->declare_or_get_parameter(
    "detection_timeout", 2.0);
  params_.rotate_to_object_timeout = node->declare_or_get_parameter(
    "rotate_to_object_timeout", 10.0);
  params_.static_object_timeout = node->declare_or_get_parameter(
    "static_object_timeout", -1.0);
  params_.linear_tolerance = node->declare_or_get_parameter(
    "linear_tolerance", 0.15);
  params_.angular_tolerance = node->declare_or_get_parameter(
    "angular_tolerance", 0.15);
  params_.max_retries = node->declare_or_get_parameter("max_retries", 3);
  params_.base_frame = node->declare_or_get_parameter(
    "base_frame", std::string("base_link"));
  params_.fixed_frame = node->declare_or_get_parameter(
    "fixed_frame", std::string("odom"));
  params_.desired_distance = node->declare_or_get_parameter(
    "desired_distance", 1.0);
  params_.skip_orientation = node->declare_or_get_parameter(
    "skip_orientation", true);
  params_.search_by_rotating = node->declare_or_get_parameter(
    "search_by_rotating", false);
  params_.search_angle = node->declare_or_get_parameter(
    "search_angle", M_PI_2);
  params_.transform_tolerance = node->declare_or_get_parameter(
    "transform_tolerance", 0.1);
  params_.odom_topic = node->declare_or_get_parameter(
    "odom_topic", std::string("odom"));
  params_.odom_duration = node->declare_or_get_parameter(
    "odom_duration", 0.3);
  params_.use_collision_detection = node->declare_or_get_parameter(
    "controller.use_collision_detection", false);
  params_.filter_coef = node->declare_or_get_parameter("filter_coef", 0.1);
  RCLCPP_INFO(logger_, "Controller frequency set to %.4fHz", params_.controller_frequency);
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
      if (parameter.as_double() < 0.0 && param_name != "static_object_timeout") {
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
      } else if (param_name == "detection_timeout") {
        params_.detection_timeout = parameter.as_double();
      } else if (param_name == "rotate_to_object_timeout") {
        params_.rotate_to_object_timeout = parameter.as_double();
      } else if (param_name == "static_object_timeout") {
        params_.static_object_timeout = parameter.as_double();
      } else if (param_name == "linear_tolerance") {
        params_.linear_tolerance = parameter.as_double();
      } else if (param_name == "angular_tolerance") {
        params_.angular_tolerance = parameter.as_double();
      } else if (param_name == "desired_distance") {
        params_.desired_distance = parameter.as_double();
      } else if (param_name == "transform_tolerance") {
        params_.transform_tolerance = parameter.as_double();
      } else if (param_name == "search_angle") {
        params_.search_angle = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_STRING) {
      if (param_name == "base_frame") {
        params_.base_frame = parameter.as_string();
      } else if (param_name == "fixed_frame") {
        params_.fixed_frame = parameter.as_string();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == "skip_orientation") {
        params_.skip_orientation = parameter.as_bool();
      } else if (param_name == "search_by_rotating") {
        params_.search_by_rotating = parameter.as_bool();
      }
    }
  }
}

}  // namespace opennav_following
