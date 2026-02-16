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

#include "nav2_theta_star_planner/parameter_handler.hpp"
#include "nav2_core/controller_exceptions.hpp"

namespace nav2_theta_star_planner
{

using nav2::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  const nav2::LifecycleNode::SharedPtr & node,
  std::string & plugin_name, const rclcpp::Logger & logger)
: nav2_util::ParameterHandler<Parameters>(node, logger)
{
  plugin_name_ = plugin_name;

  params_.how_many_corners = node->declare_or_get_parameter(
    plugin_name_ + ".how_many_corners", 8);
  if (params_.how_many_corners != 8 && params_.how_many_corners != 4) {
    params_.how_many_corners = 8;
    RCLCPP_WARN(logger_, "Your value for - .how_many_corners  was overridden, and is now set to 8");
  }
  params_.allow_unknown = node->declare_or_get_parameter(
    plugin_name_ + ".allow_unknown", true);
  params_.w_euc_cost = node->declare_or_get_parameter(
    plugin_name_ + ".w_euc_cost", 1.0);
  params_.w_traversal_cost = node->declare_or_get_parameter(
    plugin_name_ + ".w_traversal_cost", 2.0);
  params_.w_heuristic_cost = params_.w_euc_cost < 1.0 ? params_.w_euc_cost : 1.0;
  params_.terminal_checking_interval = node->declare_or_get_parameter(
    plugin_name_ + ".terminal_checking_interval", 5000);
  params_.use_final_approach_orientation = node->declare_or_get_parameter(
    plugin_name_ + ".use_final_approach_orientation", false);

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
      if (parameter.as_double() <= 0.0) {
        RCLCPP_WARN(
        logger_, "The value of parameter '%s' is incorrectly set to %f, "
        "it should be >0. Ignoring parameter update.",
        param_name.c_str(), parameter.as_double());
        result.successful = false;
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == plugin_name_ + ".how_many_corners" &&
        parameter.as_int() != 4 && parameter.as_int() != 8)
      {
        RCLCPP_WARN(
        logger_, "The value of parameter '%s' is incorrectly set to %ld, "
        "it should be either 4 or 8. Ignoring parameter update.",
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
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == plugin_name_ + ".how_many_corners") {
        params_.how_many_corners = parameter.as_int();
      }
      if (param_name == plugin_name_ + ".terminal_checking_interval") {
        params_.terminal_checking_interval = parameter.as_int();
      }
    } else if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".w_euc_cost") {
        params_.w_euc_cost = parameter.as_double();
      } else if (param_name == plugin_name_ + ".w_traversal_cost") {
        params_.w_traversal_cost = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == plugin_name_ + ".use_final_approach_orientation") {
        params_.use_final_approach_orientation = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".allow_unknown") {
        params_.allow_unknown = parameter.as_bool();
      }
    }
  }
}

}  // namespace nav2_theta_star_planner
