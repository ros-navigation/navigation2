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

#include "nav2_planner/parameter_handler.hpp"

namespace nav2_planner
{

using nav2::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  const nav2::LifecycleNode::SharedPtr & node,
  const rclcpp::Logger & logger)
: nav2_util::ParameterHandler<Parameters>(node, logger)
{
  params_.planner_ids = node->declare_or_get_parameter("planner_plugins", default_ids_);
  double expected_planner_frequency = node->declare_or_get_parameter(
    "expected_planner_frequency", 1.0);
  if (expected_planner_frequency > 0) {
    params_.max_planner_duration = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      logger_,
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrun warning messages", expected_planner_frequency);
    params_.max_planner_duration = 0.0;
  }
  double costmap_update_timeout_dbl = node->declare_or_get_parameter(
    "costmap_update_timeout", 1.0);
  params_.costmap_update_timeout = rclcpp::Duration::from_seconds(costmap_update_timeout_dbl);
  params_.partial_plan_allowed = node->declare_or_get_parameter("allow_partial_planning", false);

  if (params_.planner_ids == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin", rclcpp::ParameterValue(default_types_[i]));
    }
  }

  params_.planner_types.resize(params_.planner_ids.size());

  for (size_t i = 0; i != params_.planner_ids.size(); i++) {
    try {
      params_.planner_types[i] = nav2::get_plugin_type_param(
        node, params_.planner_ids[i]);
    } catch (const std::exception & ex) {
      throw std::runtime_error(
        "Failed to get plugin type for planner " + params_.planner_ids[i] + ". Exception: " +
        ex.what());
    }
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
    // If we are trying to change the parameter of a plugin we can just skip it at this point
    // as they handle parameter changes themselves and don't need to lock the mutex
    if (param_name.find('.') != std::string::npos) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (parameter.as_double() <= 0.0) {
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
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find('.') != std::string::npos) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "expected_planner_frequency") {
        if (parameter.as_double() > 0) {
          params_.max_planner_duration = 1 / parameter.as_double();
        }
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == "allow_partial_planning") {
        params_.partial_plan_allowed = parameter.as_bool();
      }
    }
  }
}

}  // namespace nav2_planner
