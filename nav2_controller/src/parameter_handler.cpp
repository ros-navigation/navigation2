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

#include "nav2_controller/parameter_handler.hpp"

namespace nav2_controller
{

using nav2::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  nav2::LifecycleNode::SharedPtr node,
  const rclcpp::Logger & logger)
{
  node_ = node;
  logger_ = logger;

  params_.controller_frequency = node->declare_or_get_parameter("controller_frequency", 20.0);
  params_.min_x_velocity_threshold = node->declare_or_get_parameter("min_x_velocity_threshold",
      0.0001);
  params_.min_y_velocity_threshold = node->declare_or_get_parameter("min_y_velocity_threshold",
      0.0001);
  params_.min_theta_velocity_threshold =
    node->declare_or_get_parameter("min_theta_velocity_threshold", 0.0001);
  params_.speed_limit_topic = node->declare_or_get_parameter("speed_limit_topic",
      std::string("speed_limit"));
  params_.failure_tolerance = node->declare_or_get_parameter("failure_tolerance", 0.0);
  params_.use_realtime_priority = node->declare_or_get_parameter("use_realtime_priority", false);
  params_.publish_zero_velocity = node->declare_or_get_parameter("publish_zero_velocity", true);
  params_.costmap_update_timeout = node->declare_or_get_parameter("costmap_update_timeout", 0.30);
  params_.odom_topic = node->declare_or_get_parameter("odom_topic", std::string("odom"));
  params_.odom_duration = node->declare_or_get_parameter("odom_duration", 0.3);
  params_.search_window = node->declare_or_get_parameter("search_window", 2.0);

  RCLCPP_INFO(
    logger_,
    "Controller frequency set to %.4fHz",
    params_.controller_frequency);

  RCLCPP_INFO(logger_, "getting progress checker plugins..");
  params_.progress_checker_ids = node->declare_or_get_parameter("progress_checker_plugins",
      default_progress_checker_ids_);
  if (params_.progress_checker_ids == default_progress_checker_ids_) {
    for (size_t i = 0; i < default_progress_checker_ids_.size(); ++i) {
      nav2::declare_parameter_if_not_declared(
        node, default_progress_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_progress_checker_types_[i]));
    }
  }

  RCLCPP_INFO(logger_, "getting goal checker plugins..");
  params_.goal_checker_ids = node->declare_or_get_parameter("goal_checker_plugins",
      default_goal_checker_ids_);
  if (params_.goal_checker_ids == default_goal_checker_ids_) {
    for (size_t i = 0; i < default_goal_checker_ids_.size(); ++i) {
      nav2::declare_parameter_if_not_declared(
        node, default_goal_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_goal_checker_types_[i]));
    }
  }

  RCLCPP_INFO(logger_, "getting controller plugins..");
  params_.controller_ids = node->declare_or_get_parameter("controller_plugins",
      default_controller_ids_);
  if (params_.controller_ids == default_controller_ids_) {
    for (size_t i = 0; i < default_controller_ids_.size(); ++i) {
      nav2::declare_parameter_if_not_declared(
        node, default_controller_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_controller_types_[i]));
    }
  }

  RCLCPP_INFO(logger_, "getting path handler plugins..");
  params_.path_handler_ids = node->declare_or_get_parameter("path_handler_plugins",
      default_path_handler_ids_);
  if (params_.path_handler_ids == default_path_handler_ids_) {
    for (size_t i = 0; i < default_path_handler_ids_.size(); ++i) {
      nav2::declare_parameter_if_not_declared(
        node, default_path_handler_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_path_handler_types_[i]));
    }
  }

  params_.controller_types.resize(params_.controller_ids.size());
  params_.goal_checker_types.resize(params_.goal_checker_ids.size());
  params_.progress_checker_types.resize(params_.progress_checker_ids.size());
  params_.path_handler_types.resize(params_.path_handler_ids.size());

  for (size_t i = 0; i != params_.progress_checker_ids.size(); i++) {
    try {
      params_.progress_checker_types[i] = nav2::get_plugin_type_param(
        node, params_.progress_checker_ids[i]);
    } catch (const std::exception & ex) {
      throw std::runtime_error(
        std::string("Failed to get type for progress_checker '") +
        params_.progress_checker_ids[i] + "': " + ex.what());
    }
  }

  for (size_t i = 0; i != params_.goal_checker_ids.size(); i++) {
    try {
      params_.goal_checker_types[i] =
        nav2::get_plugin_type_param(node, params_.goal_checker_ids[i]);
    } catch (const std::exception & ex) {
      throw std::runtime_error(
        std::string("Failed to get type for goal_checker '") +
        params_.goal_checker_ids[i] + "': " + ex.what());
    }
  }

  for (size_t i = 0; i != params_.controller_ids.size(); i++) {
    try {
      params_.controller_types[i] = nav2::get_plugin_type_param(node, params_.controller_ids[i]);
    } catch (const std::exception & ex) {
      throw std::runtime_error(
        std::string("Failed to get type for controller plugins '") +
        params_.controller_types[i] + "': " + ex.what());
    }
  }

  for (size_t i = 0; i != params_.path_handler_ids.size(); i++) {
    try {
      params_.path_handler_types[i] = nav2::get_plugin_type_param(node,
          params_.path_handler_ids[i]);
    } catch (const std::exception & ex) {
      throw std::runtime_error(
        std::string("Failed to get type for path handler plugins '") +
        params_.path_handler_types[i] + "': " + ex.what());
    }
  }

  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &ParameterHandler::updateParametersCallback,
      this, std::placeholders::_1));

  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ParameterHandler::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
}

ParameterHandler::~ParameterHandler()
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
rcl_interfaces::msg::SetParametersResult ParameterHandler::validateParameterUpdatesCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    // If we are trying to change the parameter of a plugin we can just skip it at this point
    // as they handle parameter changes themselves and don't need to lock the mutex
    if (param_name.find('.') != std::string::npos) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (parameter.as_double() < 0.0 && param_name != "failure_tolerance") {
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
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "min_x_velocity_threshold") {
        params_.min_x_velocity_threshold = parameter.as_double();
      } else if (param_name == "min_y_velocity_threshold") {
        params_.min_y_velocity_threshold = parameter.as_double();
      } else if (param_name == "min_theta_velocity_threshold") {
        params_.min_theta_velocity_threshold = parameter.as_double();
      } else if (param_name == "failure_tolerance") {
        params_.failure_tolerance = parameter.as_double();
      } else if (param_name == "search_window") {
        params_.search_window = parameter.as_double();
      }
    }
  }
}

}  // namespace nav2_controller
