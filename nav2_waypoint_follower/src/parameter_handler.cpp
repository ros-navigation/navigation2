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

#include "nav2_waypoint_follower/parameter_handler.hpp"

namespace nav2_waypoint_follower
{

using nav2::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  const nav2::LifecycleNode::SharedPtr & node,
  const rclcpp::Logger & logger)
: nav2_util::ParameterHandler<Parameters>(node, logger)
{
  params_.stop_on_failure = node->declare_or_get_parameter("stop_on_failure", true);
  params_.loop_rate = node->declare_or_get_parameter("loop_rate", 20);
  params_.waypoint_task_executor_id =
    node->declare_or_get_parameter("waypoint_task_executor_plugin",
    std::string("wait_at_waypoint"));
  params_.global_frame_id = node->declare_or_get_parameter("global_frame_id", std::string("map"));
  params_.waypoint_task_executor_type = nav2::get_plugin_type_param(
    node,
    params_.waypoint_task_executor_id);
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
    if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (parameter.as_int() <= 0) {
        RCLCPP_WARN(
        logger_, "The value of parameter '%s' is incorrectly set to %ld, "
        "it should be >0. Ignoring parameter update.",
        param_name.c_str(), parameter.as_int());
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
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find('.') != std::string::npos) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == "loop_rate") {
        params_.loop_rate = parameter.as_int();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == "stop_on_failure") {
        params_.stop_on_failure = parameter.as_bool();
      }
    }
  }
}

}  // namespace nav2_waypoint_follower
