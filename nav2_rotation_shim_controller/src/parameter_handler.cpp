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

#include "nav2_rotation_shim_controller/parameter_handler.hpp"

namespace nav2_rotation_shim_controller
{

using nav2::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  nav2::LifecycleNode::SharedPtr node,
  std::string & plugin_name, rclcpp::Logger & logger)
{
  node_ = node;
  plugin_name_ = plugin_name;
  logger_ = logger;

  params_.angular_dist_threshold = node->declare_or_get_parameter(plugin_name_ + ".angular_dist_threshold", 0.785); // 45 deg
  params_.angular_disengage_threshold = node->declare_or_get_parameter(plugin_name_ + ".angular_disengage_threshold", 0.785 / 2.0);
  params_.forward_sampling_distance = node->declare_or_get_parameter(plugin_name_ + ".forward_sampling_distance", 0.5);
  params_.rotate_to_heading_angular_vel = node->declare_or_get_parameter(plugin_name_ + ".rotate_to_heading_angular_vel", 1.8);
  params_.max_angular_accel = node->declare_or_get_parameter(plugin_name_ + ".max_angular_accel", 3.2);
  params_.simulate_ahead_time = node->declare_or_get_parameter(plugin_name_ + ".simulate_ahead_time", 1.0);
  params_.primary_controller = node->declare_or_get_parameter<std::string>(plugin_name_ + ".primary_controller");
  params_.rotate_to_goal_heading = node->declare_or_get_parameter(plugin_name_ + ".rotate_to_goal_heading", false);
  params_.rotate_to_heading_once = node->declare_or_get_parameter(plugin_name_ + ".rotate_to_heading_once", false);
  params_.closed_loop = node->declare_or_get_parameter(plugin_name_ + ".closed_loop", true);
  params_.use_path_orientations = node->declare_or_get_parameter(plugin_name_ + ".use_path_orientations", false);
  double control_frequency = 20.0;
  node->get_parameter("controller_frequency", control_frequency);
  params_.control_duration = 1.0 / control_frequency;
}

void ParameterHandler::activate()
{
  auto node = node_.lock();
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &ParameterHandler::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ParameterHandler::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
}

void ParameterHandler::deactivate()
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

ParameterHandler::~ParameterHandler()
{
}
rcl_interfaces::msg::SetParametersResult ParameterHandler::validateParameterUpdatesCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".simulate_ahead_time" &&
        parameter.as_double() < 0.0)
      {
        RCLCPP_WARN(
        logger_, "The value of simulate_ahead_time is incorrectly set, "
        "it should be >=0. Ignoring parameter update.");
        result.successful = false;
      } else if (parameter.as_double() <= 0.0) {
        RCLCPP_WARN(
        logger_, "The value of parameter '%s' is incorrectly set to %f, "
        "it should be >0. Ignoring parameter update.",
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
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".angular_dist_threshold") {
        params_.angular_dist_threshold = parameter.as_double();
      } else if (param_name == plugin_name_ + ".forward_sampling_distance") {
        params_.forward_sampling_distance = parameter.as_double();
      } else if (param_name == plugin_name_ + ".rotate_to_heading_angular_vel") {
        params_.rotate_to_heading_angular_vel = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_angular_accel") {
        params_.max_angular_accel = parameter.as_double();
      } else if (param_name == plugin_name_ + ".simulate_ahead_time") {
        params_.simulate_ahead_time = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == plugin_name_ + ".rotate_to_goal_heading") {
        params_.rotate_to_goal_heading = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".rotate_to_heading_once") {
        params_.rotate_to_heading_once = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".closed_loop") {
        params_.closed_loop = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".use_path_orientations") {
        params_.use_path_orientations = parameter.as_bool();
      }
    }
  }
}

}  // namespace nav2_rotation_shim_controller
