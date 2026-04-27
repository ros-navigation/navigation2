// Copyright (c) 2024 Nav2 Contributors
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
#include <memory>
#include <vector>

#include "nav2_dstar_lite_planner/parameter_handler.hpp"

namespace nav2_dstar_lite_planner
{

using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  std::string & plugin_name, const rclcpp::Logger & logger)
: logger_(logger), plugin_name_(plugin_name), node_(node)
{
  node_->declare_parameter(
    plugin_name_ + ".allow_unknown",
    rclcpp::ParameterValue(true));
  node_->declare_parameter(
    plugin_name_ + ".max_iterations",
    rclcpp::ParameterValue(100000));
  node_->declare_parameter(
    plugin_name_ + ".replan_interval",
    rclcpp::ParameterValue(10));
  node_->declare_parameter(
    plugin_name_ + ".hysteresis_factor",
    rclcpp::ParameterValue(1.05));
  node_->declare_parameter(
    plugin_name_ + ".terminal_checking_interval",
    rclcpp::ParameterValue(5000));
  node_->declare_parameter(
    plugin_name_ + ".use_final_approach_orientation",
    rclcpp::ParameterValue(false));

  params_.allow_unknown = node_->get_parameter(plugin_name_ + ".allow_unknown").as_bool();
  params_.max_iterations = node_->get_parameter(plugin_name_ + ".max_iterations").as_int();
  params_.replan_interval = node_->get_parameter(plugin_name_ + ".replan_interval").as_int();
  params_.hysteresis_factor = node_->get_parameter(plugin_name_ + ".hysteresis_factor").as_double();
  params_.terminal_checking_interval =
    node_->get_parameter(plugin_name_ + ".terminal_checking_interval").as_int();
  params_.use_final_approach_orientation =
    node_->get_parameter(plugin_name_ + ".use_final_approach_orientation").as_bool();
}

ParameterHandler::~ParameterHandler()
{
  deactivate();
}

void ParameterHandler::activate()
{
  dyn_params_handler_ = node_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return this->dynamicParametersCallback(parameters);
    });
}

void ParameterHandler::deactivate()
{
  dyn_params_handler_.reset();
}

rcl_interfaces::msg::SetParametersResult
ParameterHandler::dynamicParametersCallback(
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

    // Validate first
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".hysteresis_factor" &&
        parameter.as_double() < 1.0)
      {
        RCLCPP_WARN(
          logger_,
          "hysteresis_factor must be >= 1.0, ignoring update");
        result.successful = false;
        continue;
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == plugin_name_ + ".max_iterations" &&
        parameter.as_int() < 1 && parameter.as_int() != -1)
      {
        RCLCPP_WARN(
          logger_,
          "max_iterations must be > 0 or -1 (unlimited), ignoring update");
        result.successful = false;
        continue;
      }
      if (param_name == plugin_name_ + ".replan_interval" &&
        parameter.as_int() < 1)
      {
        RCLCPP_WARN(
          logger_,
          "replan_interval must be >= 1, ignoring update");
        result.successful = false;
        continue;
      }
    }

    // Apply update
    std::lock_guard<std::mutex> lock(mutex_);
    if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == plugin_name_ + ".allow_unknown") {
        params_.allow_unknown = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".use_final_approach_orientation") {
        params_.use_final_approach_orientation = parameter.as_bool();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == plugin_name_ + ".max_iterations") {
        params_.max_iterations = parameter.as_int();
      } else if (param_name == plugin_name_ + ".replan_interval") {
        params_.replan_interval = parameter.as_int();
      } else if (param_name == plugin_name_ + ".terminal_checking_interval") {
        params_.terminal_checking_interval = parameter.as_int();
      }
    } else if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".hysteresis_factor") {
        params_.hysteresis_factor = parameter.as_double();
      }
    }
  }

  return result;
}

}  // namespace nav2_dstar_lite_planner
