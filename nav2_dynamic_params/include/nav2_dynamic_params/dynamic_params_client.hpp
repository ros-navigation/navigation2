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

#ifndef NAV2_DYNAMIC_PARAMS_DYNAMIC_PARAMS_CLIENT_H_
#define NAV2_DYNAMIC_PARAMS_DYNAMIC_PARAMS_CLIENT_H_

#include <map>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_events_filter.hpp"

namespace nav2_dynamic_params
{

class DynamicParamsClient
{

public:

  DynamicParamsClient(std::string node_name, rclcpp::Node::SharedPtr node)
  {
    node_name_= node_name;
    //node_= node;

    // Create member node and client for node name 
    node_ = rclcpp::Node::make_shared("dynamic_params_client");
    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(node_, node_name);

  }
  template<class T>
  bool get_event_param(const rcl_interfaces::msg::ParameterEvent::SharedPtr event,
    std::string param_name, T & new_value)
  {
    rclcpp::ParameterEventsFilter filter(event, {param_name},
      {rclcpp::ParameterEventsFilter::EventType::NEW,
      rclcpp::ParameterEventsFilter::EventType::CHANGED});
    if(!(filter.get_events()).empty())
    {
      auto param_msg = ((filter.get_events()).front()).second;
      auto param_value = rclcpp::Parameter::from_parameter_msg(*param_msg);
      new_value = param_value.get_value<T>();
      return true;
    }else {
      //node_->get_parameter<T>(param_name, new_value);

      // Call through parameter client of member node
      new_value = parameters_client_->get_parameter<T>(param_name);
      return false;
    }
  }

  template<class T>
  bool get_event_param(const rcl_interfaces::msg::ParameterEvent::SharedPtr event,
    std::string param_name, T & new_value, T default_value)
  {    
    rclcpp::ParameterEventsFilter filter(event, {param_name},
      {rclcpp::ParameterEventsFilter::EventType::NEW,
      rclcpp::ParameterEventsFilter::EventType::CHANGED});
    if(!(filter.get_events()).empty())
    {
      auto param_msg = ((filter.get_events()).front()).second;
      auto param_value = rclcpp::Parameter::from_parameter_msg(*param_msg);
      new_value = param_value.get_value<T>();
      return true;
    }else {
      //node_->get_parameter_or<T>(param_name, new_value, default_value);

      // Call through parameter client of member node
      new_value = parameters_client_->get_parameter<T>(param_name, default_value);
      return false;
    }
  }
private:

  std::string node_name_;
  rclcpp::Node::SharedPtr node_; 
  rclcpp::SyncParametersClient::SharedPtr parameters_client_;

};

}  // namespace nav2_dynamic_params

#endif  // NAV2_DYNAMIC_PARAMS_DYNAMIC_PARAMS_CLIENT_H_

