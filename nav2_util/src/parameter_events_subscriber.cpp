// Copyright 2019 Intel Corporation
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

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_util/parameter_events_subscriber.hpp"

namespace nav2_util
{

ParameterEventsSubscriber::ParameterEventsSubscriber(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  const rclcpp::QoS & qos)
: node_base_(node_base),
  node_topics_(node_topics),
  node_logging_(node_logging),
  qos_(qos)
{}

void
ParameterEventsSubscriber::add_namespace_event_subscriber(const std::string & node_namespace)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (std::find(subscribed_namespaces_.begin(), subscribed_namespaces_.end(),
    node_namespace) == subscribed_namespaces_.end())
  {
    subscribed_namespaces_.push_back(node_namespace);
    auto topic = join_path(node_namespace, "parameter_events");
    RCLCPP_DEBUG(node_logging_->get_logger(), "Subscribing to topic: %s", topic.c_str());

    auto event_sub = rclcpp::create_subscription<rcl_interfaces::msg::ParameterEvent>(
      node_topics_, topic, qos_,
      std::bind(&ParameterEventsSubscriber::event_callback, this, std::placeholders::_1));

    event_subscriptions_.push_back(event_sub);
  }
}

void
ParameterEventsSubscriber::remove_namespace_event_subscriber(const std::string & node_namespace)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto check_sub_cb = [this, &node_namespace](
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr sub) {
      return sub->get_topic_name() == join_path(node_namespace, "parameter_events");
    };

  auto it = std::find_if(
    event_subscriptions_.begin(),
    event_subscriptions_.end(),
    check_sub_cb);

  if (it != event_subscriptions_.end()) {
    event_subscriptions_.erase(it);
    subscribed_namespaces_.erase(
      std::remove(subscribed_namespaces_.begin(), subscribed_namespaces_.end(), node_namespace));
  }
}

void
ParameterEventsSubscriber::set_event_callback(
  std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr &)> callback,
  const std::vector<std::string> & node_namespaces)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  remove_event_callback();
  std::string full_namespace;
  for (auto & ns : node_namespaces) {
    if (ns == "") {
      full_namespace = node_base_->get_namespace();
    } else {
      full_namespace = resolve_path(ns);
    }
    add_namespace_event_subscriber(full_namespace);
    event_namespaces_.push_back(full_namespace);
  }

  event_callback_ = callback;
}

void
ParameterEventsSubscriber::remove_event_callback()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // Clear current vector of event namespaces and remove subscriptions
  auto temp_namespaces = event_namespaces_;
  event_namespaces_.clear();
  for (auto temp_ns : temp_namespaces) {
    if (should_unsubscribe_to_namespace(temp_ns)) {
      remove_namespace_event_subscriber(temp_ns);
    }
  }

  event_callback_ = nullptr;
}

ParameterEventsCallbackHandle::SharedPtr
ParameterEventsSubscriber::add_parameter_callback(
  const std::string & parameter_name,
  ParameterEventsCallbackType callback,
  const std::string & node_name)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto full_node_name = resolve_path(node_name);
  add_namespace_event_subscriber(split_path(full_node_name).first);

  auto handle = std::make_shared<ParameterEventsCallbackHandle>();
  handle->callback = callback;
  handle->parameter_name = parameter_name;
  handle->node_name = full_node_name;
  // the last callback registered is executed first.
  parameter_callbacks_[{parameter_name, full_node_name}].emplace_front(handle);

  return handle;
}

struct HandleCompare
  : public std::unary_function<ParameterEventsCallbackHandle::WeakPtr, bool>
{
  explicit HandleCompare(const ParameterEventsCallbackHandle * const base)
  : base_(base) {}
  bool operator()(const ParameterEventsCallbackHandle::WeakPtr & handle)
  {
    auto shared_handle = handle.lock();
    if (base_ == shared_handle.get()) {
      return true;
    }
    return false;
  }
  const ParameterEventsCallbackHandle * const base_;
};

void
ParameterEventsSubscriber::remove_parameter_callback(
  const ParameterEventsCallbackHandle * const handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto & container = parameter_callbacks_[{handle->parameter_name, handle->node_name}];
  auto it = std::find_if(
    container.begin(),
    container.end(),
    HandleCompare(handle));
  if (it != container.end()) {
    container.erase(it);
    if (container.empty()) {
      parameter_callbacks_.erase({handle->parameter_name, handle->node_name});
      if (should_unsubscribe_to_namespace(split_path(handle->node_name).first)) {
        remove_namespace_event_subscriber(split_path(handle->node_name).first);
      }
    }
  } else {
    throw std::runtime_error("Callback doesn't exist");
  }
}

bool
ParameterEventsSubscriber::should_unsubscribe_to_namespace(const std::string & node_namespace)
{
  auto it1 = std::find(event_namespaces_.begin(), event_namespaces_.end(), node_namespace);
  if (it1 != event_namespaces_.end()) {
    return false;
  }

  auto it2 = parameter_callbacks_.begin();
  while (it2 != parameter_callbacks_.end()) {
    if (split_path(it2->first.second).first == node_namespace) {
      return false;
    }
    ++it2;
  }
  return true;
}

void
ParameterEventsSubscriber::remove_parameter_callback(
  const std::string & parameter_name,
  const std::string & node_name)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto full_node_name = resolve_path(node_name);
  parameter_callbacks_.erase({parameter_name, full_node_name});
  if (should_unsubscribe_to_namespace(split_path(full_node_name).first)) {
    RCLCPP_INFO(node_logging_->get_logger(), "Removing namespace event subscription");
    remove_namespace_event_subscriber(split_path(full_node_name).first);
  }
}

bool
ParameterEventsSubscriber::get_parameter_from_event(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event,
  rclcpp::Parameter & parameter,
  const std::string parameter_name,
  const std::string node_name)
{
  if (event->node == node_name) {
    rclcpp::ParameterEventsFilter filter(event, {parameter_name},
      {rclcpp::ParameterEventsFilter::EventType::NEW,
        rclcpp::ParameterEventsFilter::EventType::CHANGED});
    if (!filter.get_events().empty()) {
      const auto & events = filter.get_events();
      auto param_msg = events.back().second;
      parameter = rclcpp::Parameter::from_parameter_msg(*param_msg);
      return true;
    }
  }
  return false;
}

rclcpp::Parameter
ParameterEventsSubscriber::get_parameter_from_event(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event,
  const std::string parameter_name,
  const std::string node_name)
{
  rclcpp::Parameter p(parameter_name);
  get_parameter_from_event(event, p, parameter_name, node_name);
  return p;
}

void
ParameterEventsSubscriber::event_callback(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const std::string & node_name = event->node;
  RCLCPP_DEBUG(node_logging_->get_logger(), "Parameter event received for node: %s",
    node_name.c_str());

  for (auto it = parameter_callbacks_.begin(); it != parameter_callbacks_.end(); ++it) {
    rclcpp::Parameter p;
    if (get_parameter_from_event(event, p, it->first.first, it->first.second)) {
      for (auto cb = it->second.begin(); cb != it->second.end(); ++cb) {
        auto shared_handle = cb->lock();
        if (nullptr != shared_handle) {
          shared_handle->callback(p);
        } else {
          cb = it->second.erase(cb);
        }
      }
    }
  }

  if (event_callback_) {
    event_callback_(event);
  }
}

std::string
ParameterEventsSubscriber::resolve_path(const std::string & path)
{
  std::string full_path;

  if (path == "") {
    full_path = node_base_->get_fully_qualified_name();
  } else {
    full_path = path;
    if (*full_path.begin() != '/') {
      full_path = join_path(node_base_->get_namespace(), full_path);
    }
  }

  return full_path;
}

std::pair<std::string, std::string>
ParameterEventsSubscriber::split_path(const std::string & str)
{
  std::string path;
  std::size_t found = str.find_last_of("/\\");
  if (found == 0) {
    path = str.substr(0, found + 1);
  } else {
    path = str.substr(0, found);
  }
  std::string name = str.substr(found + 1);
  return {path, name};
}

std::string
ParameterEventsSubscriber::join_path(std::string path, std::string name)
{
  std::string joined_path = path;
  if (*joined_path.rbegin() != '/' && *name.begin() != '/') {
    joined_path = joined_path + "/";
  }

  return joined_path + name;
}

}  // namespace nav2_util
