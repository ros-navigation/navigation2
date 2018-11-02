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

#ifndef NAV2_DYNAMIC_PARAMS__DYNAMIC_PARAMS_CLIENT_HPP_
#define NAV2_DYNAMIC_PARAMS__DYNAMIC_PARAMS_CLIENT_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_events_filter.hpp"

namespace nav2_dynamic_params
{

class DynamicParamsClient
{
public:
  // From what I've gathered, events are scoped by namespace, not node name
  explicit DynamicParamsClient(
    rclcpp::Node::SharedPtr node, std::vector<std::string> remote_names = {""})
  : node_(node)
  {
    add_parameter_clients(remote_names);
    parameters_client_for_callback_ = std::make_shared<rclcpp::SyncParametersClient>(node_);
    event_subscription_ = parameters_client_for_callback_->on_parameter_event(
      std::bind(&DynamicParamsClient::event_callback, this, std::placeholders::_1));
  }

  ~DynamicParamsClient() {}

  void add_parameter_clients(std::vector<std::string> remote_names)
  {
    for (auto & name : remote_names) {
      auto client = std::make_shared<rclcpp::SyncParametersClient>(node_, name);
      parameters_clients_.push_back(client);
    }
  }

  // Sets user callback as a member variable.
  // Default true for init_callback to force user callback upon setting
  void set_callback(
    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> callback,
    bool init_callback = true)
  {
    user_callback_ = callback;
    if (init_callback) {
      force_callback();
    }
  }

  // Initializes list of parameters in the cached dynamic parameter map
  // with values from remote nodes
  void add_parameters(const std::vector<std::string> & param_names)
  {
    for (const auto & client : parameters_clients_) {
      auto params = client->get_parameters(param_names);
      for (const auto & param : params) {
        set_param_in_map(param);
      }
    }
  }

// Variant of add_parameters. Passing as char [] to avoid ambigous overload errors
  void add_parameters(const char param_name[])
  {
    std::string pname = param_name;
    add_parameters({pname});
  }

  // Adds all parameters currently on the remote nodes and initializes them
  // in the dynamic parameter map
  void add_parameters()
  {
    std::vector<std::string> param_names;
    for (const auto & client : parameters_clients_) {
      auto param_list = client->list_parameters({}, 1);
      param_names.insert(param_names.end(), param_list.names.begin(), param_list.names.end());
    }
    add_parameters(param_names);
  }

  std::vector<std::string> get_param_names()
  {
    std::vector<std::string> names;
    for (const auto & entry : dynamic_param_map_) {
      names.push_back(entry.first);
    }
    return names;
  }

  std::map<std::string, rclcpp::Parameter> get_param_map()
  {
    return dynamic_param_map_;
  }

  // This function assigns the value from the event, if present, or the dynamic parameters map,
  // otherwise it does not assign anything
  template<class T>
  bool get_event_param(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event,
    const std::string & param_name, T & new_value)
  {
    rclcpp::ParameterEventsFilter filter(event, {param_name},
      {rclcpp::ParameterEventsFilter::EventType::NEW,
        rclcpp::ParameterEventsFilter::EventType::CHANGED});
    if (!(filter.get_events()).empty()) {
      new_value = get_param_from_event<T>(filter);
      return true;
    } else {
      if (get_param_from_map<T>(param_name, new_value)) {
        return true;
      } else {
        RCLCPP_WARN(node_->get_logger(), "Parameter '%s' not set on node", param_name.c_str());
        return false;
      }
    }
  }

  // This function assigns the value from the event, if present, or the dynamic parameters map,
  // otherwise it assigns the default value
  template<class T>
  bool get_event_param_or(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr & event,
    const std::string & param_name, T & new_value, const T & default_value)
  {
    if (get_event_param<T>(event, param_name, new_value)) {
      return true;
    } else {
      new_value = default_value;
      return false;
    }
  }

  // A check to filter whether parameter name is part of the event
  bool is_in_event(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr & event, const std::string & name)
  {
    rclcpp::ParameterEventsFilter filter(event, {name},
      {rclcpp::ParameterEventsFilter::EventType::NEW,
        rclcpp::ParameterEventsFilter::EventType::CHANGED});
    return !filter.get_events().empty();
  }

  // Passes an empty event to the user_callback
  void force_callback()
  {
    auto event = std::make_shared<rcl_interfaces::msg::ParameterEvent>();
    user_callback_(event);
  }

private:
  void set_param_in_map(rclcpp::Parameter param)
  {
    if (!dynamic_param_map_.count(param.get_name())) {
      dynamic_param_map_[param.get_name()] = param;
    } else {
      if (dynamic_param_map_[param.get_name()].get_type() ==
        rclcpp::ParameterType::PARAMETER_NOT_SET)
      {
        dynamic_param_map_[param.get_name()] = param;
      }
    }
  }

  template<class T>
  T get_param_from_event(const rclcpp::ParameterEventsFilter & event_filter)
  {
    auto param_msg = ((event_filter.get_events()).front()).second;
    auto param = rclcpp::Parameter::from_parameter_msg(*param_msg);
    dynamic_param_map_[param.get_name()] = param;
    return param.get_value<T>();
  }

  template<class T>
  bool get_param_from_map(const std::string & name, T & value)
  {
    if (dynamic_param_map_.count(name) > 0 &&
      !dynamic_param_map_[name].get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      value = dynamic_param_map_[name].get_value<T>();
      return true;
    } else {
      return false;
    }
  }
  // This event callbak filters for parameters that exist in dynamic parameters map
  void event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {
    if (is_event_in_map(event)) {
      user_callback_(event);
    }
  }

  // This function checks that event variables exist in the cached dynamic param map
  // True if at least one parameter exists in the map
  bool is_event_in_map(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {
    for (auto & new_parameter : event->new_parameters) {
      if (dynamic_param_map_.count(new_parameter.name)) {
        return true;
      }
    }

    for (auto & changed_parameter : event->changed_parameters) {
      if (dynamic_param_map_.count(changed_parameter.name)) {
        return true;
      }
    }

    for (auto & deleted_parameter : event->deleted_parameters) {
      if (dynamic_param_map_.count(deleted_parameter.name)) {
        return true;
      }
    }
    return false;
  }
  // Cached Map of dynamic parameters. Parameter values are initialized
  // from remote nodes if the parameter exists
  std::map<std::string, rclcpp::Parameter> dynamic_param_map_;

  // Vector to store parameter clients to remote nodes
  std::vector<rclcpp::SyncParametersClient::SharedPtr> parameters_clients_;

  rclcpp::SyncParametersClient::SharedPtr parameters_client_for_callback_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr event_subscription_;

  // Users of this class will pass in an event callback
  std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> user_callback_;
};

}  // namespace nav2_dynamic_params

#endif  // NAV2_DYNAMIC_PARAMS__DYNAMIC_PARAMS_CLIENT_HPP_
