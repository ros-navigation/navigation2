// Copyright (c) 2018 Intel Corporation
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

#include <map>
#include <string>
#include <utility>
#include <vector>
#include <limits>
#include "nav2_dynamic_params/dynamic_params_validator.hpp"

namespace nav2_dynamic_params
{

DynamicParamsValidator::DynamicParamsValidator(rclcpp::Node::SharedPtr node, bool reject_new_params)
: node_(node),
  reject_new_params_(reject_new_params)
{
  node_->register_param_change_callback(
    std::bind(&DynamicParamsValidator::param_validation_callback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult DynamicParamsValidator::param_validation_callback(
  std::vector<rclcpp::Parameter> parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  for (auto parameter : parameters) {
    RCLCPP_INFO(node_->get_logger(),
      "Parameter Change Request: %s", (parameter.get_name()).c_str());

    // Check if parameter is static
    if (check_if_static(parameter)) {
      result.successful = false;
      return result;
    }

    // Validate Parameter Type
    if (param_map_.count(parameter.get_name()) > 0) {
      if (!validate_param(parameter)) {
        result.successful = false;
        return result;
      }
    } else {
      // Default to accept new parameters
      if (reject_new_params_) {
        result.successful = false;
        RCLCPP_WARN(node_->get_logger(),
          "Parameter Change Denied::Parameter Not Registered: %s",
          parameter.get_name().c_str());
        return result;
      }
    }

    // Validate Parameter Bounds
    if (param_bound_map_.count(parameter.get_name()) > 0) {
      if (!validate_param_bounds(parameter)) {
        result.successful = false;
        return result;
      }
    }
  }

  // Check Custom Validation Callback if provided
  if (validation_callback_) {
    return validation_callback_(parameters);
  }

  return result;
}

void DynamicParamsValidator::add_param(
  const std::string & param_name, const rclcpp::ParameterType & type)
{
  param_map_[param_name] = type;
}

void DynamicParamsValidator::add_param(
  const std::string & param_name, const rclcpp::ParameterType & type,
  std::pair<double, double> bounds)
{
  param_map_[param_name] = type;
  param_bound_map_[param_name] = bounds;
}

void DynamicParamsValidator::add_param(
  const std::string & param_name, const rclcpp::ParameterType & type,
  std::pair<double, double> bounds, const int & ignore_bound)
{
  param_map_[param_name] = type;
  if (ignore_bound) {
    bounds.second = std::numeric_limits<double>::infinity();
  } else {
    bounds.first = -std::numeric_limits<double>::infinity();
  }
  param_bound_map_[param_name] = bounds;
}

void DynamicParamsValidator::add_param(const std::map<std::string, rclcpp::ParameterType> & map)
{
  param_map_.insert(map.begin(), map.end());
}

void DynamicParamsValidator::add_static_params(std::vector<std::string> param_names)
{
  static_params_.insert(static_params_.end(), param_names.begin(), param_names.end());
}

void DynamicParamsValidator::set_validation_callback(
  std::function<rcl_interfaces::msg::SetParametersResult(
    const std::vector<rclcpp::Parameter> &)> callback)
{
  validation_callback_ = callback;
}

bool DynamicParamsValidator::validate_param(const rclcpp::Parameter & param)
{
  if (param_map_[param.get_name()] == param.get_type()) {
    return true;
  } else {
    RCLCPP_WARN(node_->get_logger(),
      "Parameter Change Denied::Doesn't Match Type: %s", param.get_name().c_str());
    return false;
  }
}

bool DynamicParamsValidator::validate_param_bounds(const rclcpp::Parameter & param)
{
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return check_bound_of_type<rclcpp::ParameterType::PARAMETER_DOUBLE>(param);
  }

  if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return check_bound_of_type<rclcpp::ParameterType::PARAMETER_INTEGER>(param);
  }
  RCLCPP_WARN(node_->get_logger(),
    "Parameter Change Denied:: Can Only Check Bounds of Type int or double: %s",
    param.get_name().c_str());
  return false;
}

bool DynamicParamsValidator::check_if_static(const rclcpp::Parameter & param)
{
  if (std::find(static_params_.begin(), static_params_.end(),
    param.get_name()) != static_params_.end())
  {
    RCLCPP_WARN(node_->get_logger(),
      "Parameter Change Denied::Parameter is static: %s", param.get_name().c_str());
    return true;
  } else {
    return false;
  }
}


}  // namespace nav2_dynamic_params
