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

#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_dynamic_params
{

class DynamicParamsClient
{
public:
  explicit DynamicParamsClient(
    rclcpp::Node::SharedPtr node)
  : node_(node),
    last_event_(std::make_shared<rcl_interfaces::msg::ParameterEvent>())
  {}

  ~DynamicParamsClient() {}

  // Adds a subscription to a namespace parameter events topic
  void add_namespace_event_subscriber(const std::string & node_namespace)
  {
    if (std::find(node_namespaces_.begin(), node_namespaces_.end(),
      node_namespace) == node_namespaces_.end())
    {
      node_namespaces_.push_back(node_namespace);
      auto topic = join_path(node_namespace, "parameter_events");
      RCLCPP_INFO(node_->get_logger(), "Subscribing to topic: %s", topic.c_str());

      // Internal callback for parameter events.
      // Namespace is passed through to resolve duplicate parameters
      std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> callback =
        [this, node_namespace](
        const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
        {
          last_event_ = event;
          if (is_event_in_map(event, node_namespace)) {
            user_callback_();
          }
        };

      auto event_sub = node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        topic, callback);
      event_subscriptions_.push_back(event_sub);
      // TODO(bpwilcox): Fix hanging problem for list parameters service call
      // get_param_list(node_namespace);
    }
  }

  // Sets user callback as a member variable.
  // Default true for init_callback to force user callback upon setting
  void set_callback(
    std::function<void()> callback,
    bool init_callback = true)
  {
    user_callback_ = callback;
    if (init_callback) {
      user_callback_();
    }
  }

  // Adds and initialize parameters to cached map of dynamic parameters
  // If an empty vector is passed, will grab all current parameters on node
  void add_parameters(const std::string & path, const std::vector<std::string> & param_names)
  {
    auto full_path = path;
    if (*full_path.begin() != '/') {
      full_path = '/' + full_path;
    }

    std::vector<rclcpp::Parameter> params;

    if (full_path == join_path(node_->get_namespace(), node_->get_name())) {
      if (param_names.size() < 1) {
        auto param_list = node_->list_parameters({}, 1);
        params = node_->get_parameters(param_list.names);
      } else {
        params = node_->get_parameters(param_names);
      }
    } else {
      auto client = std::make_shared<rclcpp::SyncParametersClient>(node_, full_path);
      if (client->wait_for_service(100ms)) {
        if (param_names.size() < 1) {
          auto param_list = client->list_parameters({}, 1);
          params = client->get_parameters(param_list.names);
        } else {
          params = client->get_parameters(param_names);
        }
      } else {
        RCLCPP_WARN(node_->get_logger(),
          "Node '%s' not available for service, inserting params as NOT SET", full_path.c_str());
        // Set param_names as PARAMETER_NOT_SET
        for (const auto & name : param_names) {
          auto param = rclcpp::Parameter(name, rclcpp::ParameterValue());
          params.push_back(param);
        }
      }
    }

    std::string node_namespace = split_path(full_path).first;
    add_namespace_event_subscriber(node_namespace);
    for (const auto & param : params) {
      init_param_in_map(param, node_namespace);
    }
  }

  // Variant of add_parameters to add parameters of member node
  // As a default (no argument), it will add all parameters on the node
  void add_parameters(const std::vector<std::string> & param_names = {})
  {
    auto full_path = join_path(node_->get_namespace(), node_->get_name());
    add_parameters(full_path, param_names);
  }

  // Variant of add_parameters to pass in namespace and node name
  void add_parameters(
    const std::string & name_space,
    const std::string & node_name, const std::vector<std::string> & param_names)
  {
    auto full_path = join_path(name_space, node_name);
    add_parameters(full_path, param_names);
  }

  // Passes empty vector to add_parameters (which will add all parameters on node)
  void add_parameters_on_node(const std::string full_path)
  {
    std::vector<std::string> empty_vector;
    add_parameters(full_path, empty_vector);
  }

  // Variant of add_parameters_on_node to include node namespace and node name
  void add_parameters_on_node(const std::string & ns, const std::string & node)
  {
    auto full_path = join_path(ns, node);
    add_parameters_on_node(full_path);
  }


  // Get list of cached dynamic param names
  std::vector<std::string> get_param_names()
  {
    std::vector<std::string> names;
    for (const auto & entry : dynamic_param_map_) {
      names.push_back(entry.first);
    }
    return names;
  }

  // return full map of dynamic parameters
  std::map<std::string, rclcpp::Parameter> get_param_map()
  {
    return dynamic_param_map_;
  }

  // retreive parameter from map
  template<class T>
  bool get_event_param(
    const std::string & name_space, const std::string & param_name, T & new_value)
  {
    auto lookup_name = join_path(name_space, param_name);
    if (get_param_from_map<T>(lookup_name, new_value)) {
      return true;
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "Parameter '%s' is either unregistered or not set", lookup_name.c_str());
      return false;
    }
  }

  // Variant of get_event_param for full path to parameter
  template<class T>
  bool get_event_param(const std::string & param_name, T & new_value)
  {
    return get_event_param<T>(node_->get_namespace(), param_name, new_value);
  }

  // retreive parameter or assign default value if not set
  template<class T>
  bool get_event_param_or(
    const std::string & name_space,
    const std::string & param_name, T & new_value, const T & default_value)
  {
    if (get_event_param<T>(name_space, param_name, new_value)) {
      return true;
    } else {
      new_value = default_value;
      return false;
    }
  }

  // Variant of get_event_param_or for full path to parameter
  template<class T>
  bool get_event_param_or(const std::string & param_name, T & new_value, const T & default_value)
  {
    return get_event_param_or<T>(node_->get_namespace(), param_name, new_value, default_value);
  }

  // A check to filter whether parameter name is part of the lastest event
  bool is_in_event(const std::string & name)
  {
    rclcpp::ParameterEventsFilter filter(last_event_, {name},
      {rclcpp::ParameterEventsFilter::EventType::NEW,
        rclcpp::ParameterEventsFilter::EventType::CHANGED});
    return !filter.get_events().empty();
  }

private:
  void init_param_in_map(rclcpp::Parameter param, std::string node_namespace)
  {
    auto param_name = join_path(node_namespace, param.get_name());
    if (!is_duplicate(param_name)) {
      if (!dynamic_param_map_.count(param_name)) {
        dynamic_param_map_[param_name] = param;
      } else {
        std::string error = "Cannot track duplicate dynamic parameters in same namespace";
        RCLCPP_ERROR(node_->get_logger(), error + ": %s", param_name.c_str());
        throw std::runtime_error(error);
      }
    } else {
      std::string error = "Duplicate parameter already exists within namespace, cannote add";
      RCLCPP_ERROR(node_->get_logger(), error + ": %s", param_name.c_str());
      throw std::runtime_error(error);
    }
  }

  // Finds all parameters that exist on available nodes in namespace
  void get_param_list(std::string ns)
  {
    auto node_list = node_->get_node_names();
    auto it = std::unique(node_list.begin(), node_list.end());
    node_list.resize(std::distance(node_list.begin(), it));
    for (const auto & node_name : node_list) {
      if (node_name == "" || *node_name.begin() == '_') {
        continue;
      }
      auto client = std::make_shared<rclcpp::SyncParametersClient>(node_, join_path(ns, node_name));
      if (client->wait_for_service(10ms)) {
        auto param_list = client->list_parameters({}, 1);
        for (auto & param_name : param_list.names) {
          param_name = join_path(ns, param_name);
        }
        all_param_list_.insert(
          all_param_list_.end(), param_list.names.begin(), param_list.names.end());
      }
    }
  }

  std::pair<std::string, std::string> split_path(const std::string & str)
  {
    std::size_t found = str.find_last_of("/\\");
    std::string path = str.substr(0, found);
    std::string name = str.substr(found + 1);
    return {path, name};
  }

  std::string join_path(std::string path, std::string name)
  {
    std::string joined_path = path;
    if (*joined_path.rbegin() != '/' && *name.begin() != '/') {
      joined_path = joined_path + "/";
    }
    if (*joined_path.begin() != '/') {
      joined_path = "/" + joined_path;
    }
    joined_path = joined_path + name;
    return joined_path;
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
  // Check if full path of parameter name is duplicate
  bool is_duplicate(std::string param_name)
  {
    if (std::count(all_param_list_.begin(), all_param_list_.end(), param_name) > 1) {
      return true;
    } else {
      return false;
    }
  }

  // This function checks that event variables exist in the cached dynamic param map
  // True if at least one parameter exists in the map
  bool is_event_in_map(const rcl_interfaces::msg::ParameterEvent::SharedPtr event, std::string ns)
  {
    bool result = false;

    for (auto & new_parameter : event->new_parameters) {
      auto param_name = join_path(ns, new_parameter.name);
      if (dynamic_param_map_.count(param_name)) {
        auto param = rclcpp::Parameter::from_parameter_msg(new_parameter);
        dynamic_param_map_[param_name] = param;
        result = true;
      }
    }

    for (auto & changed_parameter : event->changed_parameters) {
      auto param_name = join_path(ns, changed_parameter.name);
      if (dynamic_param_map_.count(param_name)) {
        auto param = rclcpp::Parameter::from_parameter_msg(changed_parameter);
        if (param.get_type() == dynamic_param_map_[param_name].get_type()) {
          dynamic_param_map_[param_name] = param;
          result = true;
        } else {
          RCLCPP_WARN(node_->get_logger(),
            "Un-matching type for parameter event: %s", param_name.c_str());
        }
      }
    }

    for (auto & deleted_parameter : event->deleted_parameters) {
      if (dynamic_param_map_.count(join_path(ns, deleted_parameter.name))) {
        result = true;
      }
    }
    return result;
  }

  // Cached Map of dynamic parameters. Parameter values are initialized
  // from remote nodes if the parameter exists
  std::map<std::string, rclcpp::Parameter> dynamic_param_map_;

  // Map to store parameter clients to remote nodes
  std::map<std::string, rclcpp::SyncParametersClient::SharedPtr> parameters_clients_;

  rclcpp::Node::SharedPtr node_;

  // Vector of unique namespaces added
  std::vector<std::string> node_namespaces_;

  // List of all known parameters on each namespace
  std::vector<std::string> all_param_list_;

  // vector of event subscriptions for each namespace
  std::vector<rclcpp::Subscription
    <rcl_interfaces::msg::ParameterEvent>::SharedPtr> event_subscriptions_;

  // Users of this class will pass in an event callback
  std::function<void()> user_callback_;

  // Pointer to latest event message
  rcl_interfaces::msg::ParameterEvent::SharedPtr last_event_;
};

}  // namespace nav2_dynamic_params

#endif  // NAV2_DYNAMIC_PARAMS__DYNAMIC_PARAMS_CLIENT_HPP_
