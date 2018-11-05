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
#include <iostream>       // std::cout
#include <string>         // std::string
#include <cstddef>         // std::size_t

namespace nav2_dynamic_params
{

class DynamicParamsClient
{
public:
  explicit DynamicParamsClient(
    rclcpp::Node::SharedPtr node, std::vector<std::string> remote_names = {""})
  : node_(node)
  {
    add_parameter_clients(remote_names);
  }

  ~DynamicParamsClient() {}

  std::string resolve_namespace_params (std::string & remote_name)
  {
    if (remote_name == "") {
      std::string node_namespace = node_->get_namespace();
      return node_namespace;
    } else if (remote_name.at(0) != '/') {
      remote_name = '/' + remote_name;
    }

    auto node_namespace = split_path(remote_name).first;
    return node_namespace;
  }

  std::pair<std::string, std::string> split_path(const std::string & str)
  {
    std::size_t found = str.find_last_of("/\\");
    std::string path = str.substr(0,found);
    std::string name = str.substr(found+1);
    return {path, name};
  }

  void add_namespace_event_subscriber(const std::string & node_namespace)
  {
    if (std::find(node_namespaces_.begin(), node_namespaces_.end(),
      node_namespace) == node_namespaces_.end())
    {
      node_namespaces_.push_back(node_namespace);
      auto topic = join_path(node_namespace, "parameter_events");
      std::cout << "Subsribing to topic: " <<  topic << "\n";

    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> callback = [this, node_namespace](
        const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
        {
          if (is_event_in_map(event, node_namespace)) {
            set_events_in_map(event, node_namespace);
            user_callback_(event);
          }
        };

      auto event_sub = node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        topic, callback);

      event_subscriptions_.push_back(event_sub);  
    }
  }

  void add_parameter_clients(std::vector<std::string> & remote_names)
  {
    for (auto & name : remote_names) {
      auto node_namespace = resolve_namespace_params(name);
      add_namespace_event_subscriber(node_namespace);
      auto client = std::make_shared<rclcpp::SyncParametersClient>(node_, name);
      //parameters_clients_.push_back(client);
      parameters_clients_[name] = std::make_shared<rclcpp::SyncParametersClient>(node_, name);
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
    for (auto & client : parameters_clients_)
    {
      auto params = client.second->get_parameters(param_names);
      //std::size_t found = client.first.find_last_of("/\\");
      //std::string node_namespace = client.first.substr(0,found);
      std::string node_namespace = split_path(client.first).first;
      for (const auto & param : params) {
        init_param_in_map(param, node_namespace);
      }
    }
  }

  void add_parameters(const std::string & ns, const std::vector<std::string> & param_names)
  {
    for (auto & client : parameters_clients_)
    {
      std::size_t found = client.first.find_last_of("/\\");
      std::string node_namespace = client.first.substr(0,found);    
      if (node_namespace == ns || node_namespace == "/" + ns) {  
        auto params = client.second->get_parameters(param_names);
        for (const auto & param : params) {
          init_param_in_map(param, node_namespace);
        }        
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
      auto param_list = client.second->list_parameters({}, 1);
      param_names.insert(param_names.end(), param_list.names.begin(), param_list.names.end());
    }

    add_parameters(param_names);
  }

  std::vector<std::string> get_param_names()
  {
    std::vector<std::string> names;
    for (const auto & entry : dynamic_param_map_) {
      if (entry.second.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
        names.push_back(entry.first);
      }
    }
    return names;
  }

  std::map<std::string, rclcpp::Parameter> get_param_map()
  {
    return dynamic_param_map_;
  }

  // Alternate Versions without event 
  template<class T>
  bool get_event_param(const std::string & param_name, T & new_value, const std::string & name_space = "")
  {
    auto lookup_name = join_path(name_space, param_name);
    if (get_param_from_map<T>(lookup_name, new_value)) {
      return true;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Parameter '%s' not set on node", lookup_name.c_str());
      return false;
    }
  }

  template<class T>
  bool get_event_param_or(const std::string & param_name, T & new_value, const T & default_value, const std::string & name_space = "")
  {
    if (get_event_param<T>(param_name, new_value, name_space)) {
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
  void init_param_in_map(rclcpp::Parameter param, std::string node_namespace)
  {
    auto param_name = join_path(node_namespace,param.get_name());
    if (!dynamic_param_map_.count(param_name)) {
      dynamic_param_map_[param_name] = param;
    } else {
      if (dynamic_param_map_[param_name].get_type() ==
        rclcpp::ParameterType::PARAMETER_NOT_SET)
      {
        dynamic_param_map_[param_name] = param;
      } else if (dynamic_param_map_[param_name].get_type() !=
        rclcpp::ParameterType::PARAMETER_NOT_SET &&
        param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
        {
          RCLCPP_WARN(node_->get_logger(),
            "Duplicate Parameter Name: %s", param.get_name().c_str());
        }
    }
  }

  void set_events_in_map(const rcl_interfaces::msg::ParameterEvent::SharedPtr event, std::string ns)
  {
    for (auto & new_parameter : event->new_parameters) {
      if (dynamic_param_map_.count(join_path(ns, new_parameter.name))) {
        auto param = rclcpp::Parameter::from_parameter_msg(new_parameter);
        dynamic_param_map_[join_path(ns, new_parameter.name)] = param;
      }
    }

    for (auto & changed_parameter : event->changed_parameters) {
      if (dynamic_param_map_.count(join_path(ns, changed_parameter.name))) {
        auto param = rclcpp::Parameter::from_parameter_msg(changed_parameter);
        dynamic_param_map_[join_path(ns, changed_parameter.name)] = param;
      }
    }
  }

  std::string join_path(std::string path, std::string name)
  {
    std::string joined_path = path;
    if (*joined_path.rbegin() != '/') {     
      joined_path = joined_path + "/";
    }
    if (*joined_path.begin() != '/') {
      joined_path = "/" + joined_path;
    }
    joined_path = joined_path + name;
    return joined_path;
  }

  void set_param_in_map(rclcpp::Parameter param)
  {
    if (!dynamic_param_map_.count(param.get_name())) {
      dynamic_param_map_[param.get_name()] = param;
    } else {
      if (dynamic_param_map_[param.get_name()].get_type() ==
        rclcpp::ParameterType::PARAMETER_NOT_SET)
      {
        dynamic_param_map_[param.get_name()] = param;
      } else if (dynamic_param_map_[param.get_name()].get_type() !=
        rclcpp::ParameterType::PARAMETER_NOT_SET &&
        param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
        {
          RCLCPP_WARN(node_->get_logger(),
            "Duplicate Parameter Name: %s", param.get_name().c_str());
        }
    }
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

  // This function checks that event variables exist in the cached dynamic param map
  // True if at least one parameter exists in the map
   bool is_event_in_map(const rcl_interfaces::msg::ParameterEvent::SharedPtr event, std::string ns)
  {
    for (auto & new_parameter : event->new_parameters) {
      if (dynamic_param_map_.count(join_path(ns, new_parameter.name))) {
        return true;
      }
    }

    for (auto & changed_parameter : event->changed_parameters) {
      if (dynamic_param_map_.count(join_path(ns, changed_parameter.name))) {
        return true;
      }
    }

    for (auto & deleted_parameter : event->deleted_parameters) {
      if (dynamic_param_map_.count(join_path(ns, deleted_parameter.name))) {
        return true;
      }
    }
    return false;
  }

  // Cached Map of dynamic parameters. Parameter values are initialized
  // from remote nodes if the parameter exists
  std::map<std::string, rclcpp::Parameter> dynamic_param_map_;
  
  // Map to store parameter clients to remote nodes
  std::map<std::string, rclcpp::SyncParametersClient::SharedPtr> parameters_clients_;

  rclcpp::SyncParametersClient::SharedPtr parameters_client_for_callback_;
  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> node_namespaces_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr event_subscription_;
  std::vector<rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr> event_subscriptions_;

  // Users of this class will pass in an event callback
  std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> user_callback_;

  rclcpp::Node::SharedPtr nh_;
};

}  // namespace nav2_dynamic_params

#endif  // NAV2_DYNAMIC_PARAMS__DYNAMIC_PARAMS_CLIENT_HPP_
