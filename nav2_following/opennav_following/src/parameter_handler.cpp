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

using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  const nav2_util::LifecycleNode::SharedPtr & node,
  const rclcpp::Logger & logger)
: node_(node), logger_(logger)
{
  nav2_util::declare_parameter_if_not_declared(
    node, "controller_frequency", rclcpp::ParameterValue(50.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "detection_timeout", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "rotate_to_object_timeout", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "static_object_timeout", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "linear_tolerance", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, "angular_tolerance", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, "max_retries", rclcpp::ParameterValue(3));
  nav2_util::declare_parameter_if_not_declared(
    node, "base_frame", rclcpp::ParameterValue(std::string("base_link")));
  nav2_util::declare_parameter_if_not_declared(
    node, "fixed_frame", rclcpp::ParameterValue(std::string("odom")));
  nav2_util::declare_parameter_if_not_declared(
    node, "desired_distance", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "skip_orientation", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, "search_by_rotating", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, "search_angle", rclcpp::ParameterValue(M_PI_2));
  nav2_util::declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "odom_topic", rclcpp::ParameterValue(std::string("odom")));
  nav2_util::declare_parameter_if_not_declared(
    node, "odom_duration", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.use_collision_detection", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, "filter_coef", rclcpp::ParameterValue(0.1));

  node->get_parameter("controller_frequency", params_.controller_frequency);
  node->get_parameter("detection_timeout", params_.detection_timeout);
  node->get_parameter("rotate_to_object_timeout", params_.rotate_to_object_timeout);
  node->get_parameter("static_object_timeout", params_.static_object_timeout);
  node->get_parameter("linear_tolerance", params_.linear_tolerance);
  node->get_parameter("angular_tolerance", params_.angular_tolerance);
  node->get_parameter("max_retries", params_.max_retries);
  node->get_parameter("base_frame", params_.base_frame);
  node->get_parameter("fixed_frame", params_.fixed_frame);
  node->get_parameter("desired_distance", params_.desired_distance);
  node->get_parameter("skip_orientation", params_.skip_orientation);
  node->get_parameter("search_by_rotating", params_.search_by_rotating);
  node->get_parameter("search_angle", params_.search_angle);
  node->get_parameter("transform_tolerance", params_.transform_tolerance);
  node->get_parameter("odom_topic", params_.odom_topic);
  node->get_parameter("odom_duration", params_.odom_duration);
  node->get_parameter("controller.use_collision_detection", params_.use_collision_detection);
  node->get_parameter("filter_coef", params_.filter_coef);

  RCLCPP_INFO(logger_, "Controller frequency set to %.4fHz", params_.controller_frequency);
}

void ParameterHandler::activate()
{
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ParameterHandler::dynamicParametersCallback, this, std::placeholders::_1));
}

void ParameterHandler::deactivate()
{
  auto node = node_.lock();
  if (dyn_params_handler_ && node) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
}

rcl_interfaces::msg::SetParametersResult ParameterHandler::dynamicParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  rcl_interfaces::msg::SetParametersResult result;

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    // Skip plugin parameters
    if (param_name.find('.') != std::string::npos) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      // Validate positive-only parameters
      if (parameter.as_double() <= 0.0 &&
        (param_name == "controller_frequency" || param_name == "detection_timeout" ||
        param_name == "rotate_to_object_timeout"))
      {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %f, "
          "it should be >0. Ignoring parameter update.",
          param_name.c_str(), parameter.as_double());
        result.successful = false;
        return result;
      } else if (parameter.as_double() < 0.0 && param_name != "static_object_timeout") {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %f, "
          "it should be >=0. Ignoring parameter update.",
          param_name.c_str(), parameter.as_double());
        result.successful = false;
        return result;
      }

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

  result.successful = true;
  return result;
}

}  // namespace opennav_following
