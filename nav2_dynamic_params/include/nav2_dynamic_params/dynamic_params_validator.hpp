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

#ifndef NAV2_DYNAMIC_PARAMS__DYNAMIC_PARAMS_VALIDATOR_HPP_
#define NAV2_DYNAMIC_PARAMS__DYNAMIC_PARAMS_VALIDATOR_HPP_

#include <map>
#include <string>
#include <utility>
#include <vector>
#include "rclcpp/rclcpp.hpp"

namespace nav2_dynamic_params
{

class DynamicParamsValidator
{
public:
  explicit DynamicParamsValidator(rclcpp::Node::SharedPtr node, bool reject_new_params = false);

  ~DynamicParamsValidator() {}

  void add_param(const std::string & param_name, const rclcpp::ParameterType & type);

  void add_param(
    const std::string & param_name, const rclcpp::ParameterType & type,
    std::pair<double, double> bounds);

  void add_param(
    const std::string & param_name, const rclcpp::ParameterType & type,
    std::pair<double, double> bounds, const int & ignore_bound);

  void add_param(const std::map<std::string, rclcpp::ParameterType> & map);

  // Reject parameter changes to static parameters
  void add_static_params(std::vector<std::string> static_param_names);

  void set_validation_callback(
    std::function<rcl_interfaces::msg::SetParametersResult(
      const std::vector<rclcpp::Parameter> &)> callback);

private:
  rcl_interfaces::msg::SetParametersResult param_validation_callback(
    std::vector<rclcpp::Parameter> parameters);

  bool validate_param(const rclcpp::Parameter & param);

  bool validate_param_bounds(const rclcpp::Parameter & param);

  bool check_if_static(const rclcpp::Parameter & param);

  template<rclcpp::ParameterType ParamT>
  bool check_bound_of_type(const rclcpp::Parameter & param)
  {
    auto value = param.get_value<ParamT>();
    auto name = param.get_name();
    if (value >= param_bound_map_[name].first && value <= param_bound_map_[name].second) {
      return true;
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "Parameter Change Denied::Outside Bounds: %s", name.c_str());
      return false;
    }
  }

  rclcpp::Node::SharedPtr node_;
  bool reject_new_params_;
  std::map<std::string, rclcpp::ParameterType> param_map_;
  std::map<std::string, std::pair<double, double>> param_bound_map_;
  std::vector<std::string> static_params_;

  std::function<rcl_interfaces::msg::SetParametersResult(
      const std::vector<rclcpp::Parameter> &)> validation_callback_;
};

}  // namespace nav2_dynamic_params

#endif  // NAV2_DYNAMIC_PARAMS__DYNAMIC_PARAMS_VALIDATOR_HPP_
