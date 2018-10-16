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

#ifndef NAV2_DYNAMIC_PARAMS_DYNAMIC_PARAMS_VALIDATOR_H_
#define NAV2_DYNAMIC_PARAMS_DYNAMIC_PARAMS_VALIDATOR_H_

#include <map>
#include "rclcpp/rclcpp.hpp"

namespace nav2_dynamic_params
{

class DynamicParamsValidator
{
public:

  DynamicParamsValidator(rclcpp::Node::SharedPtr node);

  ~DynamicParamsValidator(){}

  void add_param(std::string param_name, rclcpp::ParameterType type);

  void add_param(std::string param_name, rclcpp::ParameterType type, std::pair<double,double> bounds);

  void add_param(std::string param_name, rclcpp::ParameterType type, std::pair<double,double> bounds, int ignore_bound);

  void add_param(std::map<std::string, rclcpp::ParameterType> map);

  void set_validation_callback(std::function<rcl_interfaces::msg::SetParametersResult(
    const std::vector<rclcpp::Parameter> &)> callback);
 
private:

  rcl_interfaces::msg::SetParametersResult param_validation_callback(
    std::vector<rclcpp::Parameter> parameters);

  bool validate_param(std::string param_name, rclcpp::Parameter param, rclcpp::ParameterType type);

  bool validate_param_bounds(std::string param_name, rclcpp::Parameter param, std::pair<double,double> bound);

  rclcpp::Node::SharedPtr node_;

  std::map<std::string, rclcpp::ParameterType> param_map_;
  std::map<std::string, std::pair<double,double>> param_bound_map_;

  std::function<rcl_interfaces::msg::SetParametersResult(
    const std::vector<rclcpp::Parameter> &)> validation_callback_;

  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_sub_;

};

}  // namespace nav2_dynamic_params

#endif  // NAV2_DYNAMIC_PARAMS_DYNAMIC_PARAMS_VALIDATOR_H_

