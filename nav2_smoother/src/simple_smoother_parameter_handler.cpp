// Copyright (c) 2026 Pannaga Sudarshan
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

#include <string>
#include <vector>

#include "nav2_smoother/simple_smoother_parameter_handler.hpp"

namespace nav2_smoother
{

using rcl_interfaces::msg::ParameterType;

SimpleSmootherParameterHandler::SimpleSmootherParameterHandler(
  const nav2::LifecycleNode::SharedPtr & node,
  std::string & plugin_name, rclcpp::Logger & logger)
: nav2_util::ParameterHandler<SimpleSmootherParameters>(node, logger)
{
  plugin_name_ = plugin_name;
  params_.tolerance = node->declare_or_get_parameter(plugin_name + ".tolerance", 1.0e-10);
  params_.max_its = node->declare_or_get_parameter(plugin_name + ".max_its", 1000);
  params_.w_data = node->declare_or_get_parameter(plugin_name + ".w_data", 0.2);
  params_.w_smooth = node->declare_or_get_parameter(plugin_name + ".w_smooth", 0.3);
  params_.do_refinement = node->declare_or_get_parameter(plugin_name + ".do_refinement", true);
  params_.refinement_num = node->declare_or_get_parameter(plugin_name + ".refinement_num", 2);
  params_.enforce_path_inversion = node->declare_or_get_parameter(
    plugin_name + ".enforce_path_inversion", true);
}

rcl_interfaces::msg::SetParametersResult
SimpleSmootherParameterHandler::validateParameterUpdatesCallback(
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
    if (!mutex_.try_lock()) {
      RCLCPP_WARN(
        logger_, "Unable to change parameter '%s' while smoothing is in progress. "
        "Rejecting update.", param_name.c_str());
      result.successful = false;
      return result;
    }
    mutex_.unlock();
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      const bool is_weight = param_name == plugin_name_ + ".w_data" ||
        param_name == plugin_name_ + ".w_smooth";
      if (param_name == plugin_name_ + ".tolerance" && parameter.as_double() <= 0.0) {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %f, "
          "it should be >0. Ignoring parameter update.",
          param_name.c_str(), parameter.as_double());
        result.successful = false;
      } else if (is_weight && parameter.as_double() < 0.0) {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %f, "
          "it should be >=0. Ignoring parameter update.",
          param_name.c_str(), parameter.as_double());
        result.successful = false;
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == plugin_name_ + ".max_its" && parameter.as_int() <= 0) {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %ld, "
          "it should be >0. Ignoring parameter update.",
          param_name.c_str(), parameter.as_int());
        result.successful = false;
      } else if (param_name == plugin_name_ + ".refinement_num" && parameter.as_int() < 0) {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %ld, "
          "it should be >=0. Ignoring parameter update.",
          param_name.c_str(), parameter.as_int());
        result.successful = false;
      }
    }
  }
  return result;
}

void
SimpleSmootherParameterHandler::updateParametersCallback(
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
      if (param_name == plugin_name_ + ".tolerance") {
        params_.tolerance = parameter.as_double();
      } else if (param_name == plugin_name_ + ".w_smooth") {
        params_.w_smooth = parameter.as_double();
      } else if (param_name == plugin_name_ + ".w_data") {
        params_.w_data = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == plugin_name_ + ".max_its") {
        params_.max_its = parameter.as_int();
      } else if (param_name == plugin_name_ + ".refinement_num") {
        params_.refinement_num = parameter.as_int();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == plugin_name_ + ".do_refinement") {
        params_.do_refinement = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".enforce_path_inversion") {
        params_.enforce_path_inversion = parameter.as_bool();
      }
    }
  }
}


}  // namespace nav2_smoother
