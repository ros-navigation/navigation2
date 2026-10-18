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

#include "nav2_smoother/savitzky_golay_smoother_parameter_handler.hpp"

namespace nav2_smoother
{

using rcl_interfaces::msg::ParameterType;

SavitzkyGolaySmootherParameterHandler::SavitzkyGolaySmootherParameterHandler(
  const nav2::LifecycleNode::SharedPtr & node,
  std::string & plugin_name, rclcpp::Logger & logger)
: nav2_util::ParameterHandler<SavitzkyGolaySmootherParameters>(node, logger)
{
  plugin_name_ = plugin_name;
  params_.do_refinement = node->declare_or_get_parameter(plugin_name + ".do_refinement", true);
  params_.refinement_num = node->declare_or_get_parameter(plugin_name + ".refinement_num", 2);
  params_.enforce_path_inversion = node->declare_or_get_parameter(
    plugin_name + ".enforce_path_inversion", true);
  params_.window_size = node->declare_or_get_parameter(plugin_name + ".window_size", 7);
  params_.poly_order = node->declare_or_get_parameter(plugin_name + ".poly_order", 3);
}

rcl_interfaces::msg::SetParametersResult
SavitzkyGolaySmootherParameterHandler::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // window_size and poly_order constrain each other, so work out what the pair
  // would be after this batch is applied and validate them together
  int window_size = params_.window_size;
  int poly_order = params_.poly_order;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == plugin_name_ + ".window_size") {
      window_size = parameter.as_int();
    } else if (parameter.get_name() == plugin_name_ + ".poly_order") {
      poly_order = parameter.as_int();
    }
  }
  const bool window_size_valid =
    window_size % 2 == 1 && window_size >= 3 && window_size > poly_order;
  const bool poly_order_valid = poly_order >= 0 && poly_order < window_size;

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
    if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == plugin_name_ + ".refinement_num" && parameter.as_int() < 0) {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %ld, "
          "it should be >=0. Ignoring parameter update.",
          param_name.c_str(), parameter.as_int());
        result.successful = false;
      } else if (param_name == plugin_name_ + ".window_size" && !window_size_valid) {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %d, "
          "it should be odd, at least 3, and greater than poly_order (%d). "
          "Ignoring parameter update.",
          param_name.c_str(), window_size, poly_order);
        result.successful = false;
      } else if (param_name == plugin_name_ + ".poly_order" && !poly_order_valid) {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %d, "
          "it should be >=0 and less than window_size (%d). Ignoring parameter update.",
          param_name.c_str(), poly_order, window_size);
        result.successful = false;
      }
    }
  }
  return result;
}

void
SavitzkyGolaySmootherParameterHandler::updateParametersCallback(
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
      if (param_name == plugin_name_ + ".refinement_num") {
        params_.refinement_num = parameter.as_int();
      } else if (param_name == plugin_name_ + ".window_size") {
        params_.window_size = parameter.as_int();
      } else if (param_name == plugin_name_ + ".poly_order") {
        params_.poly_order = parameter.as_int();
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
