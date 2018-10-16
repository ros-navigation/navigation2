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

#include <limits>
#include "nav2_dynamic_params/dynamic_params_validator.hpp"

namespace nav2_dynamic_params
{

DynamicParamsValidator::DynamicParamsValidator(rclcpp::Node::SharedPtr node)
: node_(node)
{
  node_->register_param_change_callback(
    std::bind(&DynamicParamsValidator::param_validation_callback, this, std::placeholders::_1));      
}

rcl_interfaces::msg::SetParametersResult DynamicParamsValidator::param_validation_callback(
  std::vector<rclcpp::Parameter> parameters)
{    
  
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  for (auto parameter : parameters){
  RCLCPP_INFO(node_->get_logger(), "Parameter Change Request: %s", (parameter.get_name()).c_str());
  
    // Validate Parameter Type
    for (std::map<std::string, rclcpp::ParameterType>::iterator it=param_map_.begin(); it!=param_map_.end(); ++it){
      if(validate_param(it->first, parameter, it->second)){
        result.successful &= true;
      } else {
        result.successful = false;
      }
    }

    // Validate Parameter Bounds
    for (std::map<std::string, std::pair<double,double>>::iterator it=param_bound_map_.begin(); it!=param_bound_map_.end(); ++it){
      if(validate_param_bounds(it->first, parameter, it->second)){
        result.successful &= true;
      } else {
        result.successful = false;
      }
    }
  }
  // Check Custom Validation Callback if provided
  if (validation_callback_){
    result.successful &= (validation_callback_(parameters)).successful;
  }

  return result;
}

void DynamicParamsValidator::add_param(std::string param_name, rclcpp::ParameterType type)
{
  param_map_[param_name] = type;
}

void DynamicParamsValidator::add_param(std::string param_name, rclcpp::ParameterType type, std::pair<double,double> bounds)
{
  param_map_[param_name] = type;
  param_bound_map_[param_name] = bounds;
}

void DynamicParamsValidator::add_param(
  std::string param_name, rclcpp::ParameterType type, std::pair<double,double> bounds, int ignore_bound)
{
  param_map_[param_name] = type;
  if (ignore_bound){
    bounds.second = std::numeric_limits<double>::infinity();
  }
  else{
    bounds.first = -std::numeric_limits<double>::infinity();
  }
  param_bound_map_[param_name] = bounds;
}

void DynamicParamsValidator::add_param(std::map<std::string, rclcpp::ParameterType> map)
{
  param_map_.insert(map.begin(), map.end());
}

void DynamicParamsValidator::set_validation_callback(std::function<rcl_interfaces::msg::SetParametersResult(
  const std::vector<rclcpp::Parameter> &)> callback) 
  {
    validation_callback_ = callback;
  }

bool DynamicParamsValidator::validate_param(std::string param_name, rclcpp::Parameter param, rclcpp::ParameterType type)
{
if (param.get_name()==param_name){
  if(type == param.get_type()){
    RCLCPP_INFO(node_->get_logger(), "Parameter Change Successful: %s", param_name.c_str());    
    return true;
  }
  else{
    RCLCPP_WARN(node_->get_logger(), "Parameter Change Denied::Doesn't Match Type: %s", param_name.c_str());    
    return false;
  }
}
return true;
}

bool DynamicParamsValidator::validate_param_bounds(std::string param_name, rclcpp::Parameter param, std::pair<double,double> bound)
{
  if (param.get_name()==param_name){
    if (param.get_type()==rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      if(param.get_value<rclcpp::ParameterType::PARAMETER_DOUBLE>() >= bound.first &&
        param.get_value<rclcpp::ParameterType::PARAMETER_DOUBLE>() <= bound.second){
        return true;        
      } else{
          RCLCPP_WARN(node_->get_logger(), "Parameter Change Denied::Outside Bounds: %s", param_name.c_str());              
          return false;
      }
    }
    if (param.get_type()==rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      if(param.get_value<rclcpp::ParameterType::PARAMETER_INTEGER>() >= bound.first &&
        param.get_value<rclcpp::ParameterType::PARAMETER_INTEGER>() <= bound.second){
        return true;        
      } else{
          RCLCPP_WARN(node_->get_logger(), "Parameter Change Denied::Outside Bounds: %s", param_name.c_str());              
          return false;
      }
    }            
    return false;
  }
  return true;
}

}  // namespace nav2_dynamic_params
