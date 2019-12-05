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

#ifndef NAV2_UTIL__PARAMETER_EVENTS_SUBSCRIBER_HPP_
#define NAV2_UTIL__PARAMETER_EVENTS_SUBSCRIBER_HPP_

#include <string>
#include <utility>
#include <unordered_map>
#include <vector>
#include <list>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_events_filter.hpp"

namespace nav2_util
{

struct ParameterEventsCallbackHandle
{
  RCLCPP_SMART_PTR_DEFINITIONS(ParameterEventsCallbackHandle)

  using ParameterEventsCallbackType = std::function<void (const rclcpp::Parameter &)>;

  std::string parameter_name;
  std::string node_name;
  ParameterEventsCallbackType callback;
};

class ParameterEventsSubscriber
{
public:
  ParameterEventsSubscriber(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const rclcpp::QoS & qos = rclcpp::ParameterEventsQoS());

  template<typename NodeT>
  ParameterEventsSubscriber(
    NodeT node,
    const rclcpp::QoS & qos =
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_parameter_events)))
  : ParameterEventsSubscriber(
      node->get_node_base_interface(),
      node->get_node_topics_interface(),
      node->get_node_logging_interface(),
      qos)
  {}

  /// Set a custom callback for parameter events.
  /**
   * If no namespace is provided, a subscription will be created for the current namespace.
   * Repeated calls to this function will overwrite the callback.
   * If more than one namespace already has a subscription to its parameter events topic, then the
   * provided callback will be applied to all of them.
   *
   * \param[in] callback Function callback to be evaluated on event.
   * \param[in] node_namespaces Vector of namespaces for which a subscription will be created.
   */
  void
  set_event_callback(
    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr &)> callback,
    const std::vector<std::string> & node_namespaces = {""});

  /// Remove parameter event callback.
  /**
   * Calling this function will set the event callback to nullptr. This function will also remove
   * event subscriptions on the namespaces for which there are no other callbacks (from parameter
   * callbacks) active on that namespace.
   */
  void
  remove_event_callback();

  using ParameterEventsCallbackType = ParameterEventsCallbackHandle::ParameterEventsCallbackType;

  /// Add a custom callback for a specified parameter.
  /**
   * If a node_name is not provided, defaults to the current node.
   *
   * \param[in] parameter_name Name of parameter.
   * \param[in] callback Function callback to be evaluated upon parameter event.
   * \param[in] node_name Name of node which hosts the parameter.
   */
  ParameterEventsCallbackHandle::SharedPtr
  add_parameter_callback(
    const std::string & parameter_name,
    ParameterEventsCallbackType callback,
    const std::string & node_name = "");

  /// Remove a custom callback for a specified parameter given its callback handle.
  /**
   * The parameter name and node name are inspected from the callback handle. The callback handle
   * is erased from the list of callback handles on the {parameter_name, node_name} in the map.
   * An error is thrown if the handle does not exist and/or was already removed.
   *
   * \param[in] handle Pointer to callback handle to be removed.
   */
  void
  remove_parameter_callback(
    const ParameterEventsCallbackHandle * const handle);

  /// Remove a custom callback for a specified parameter given its name and respective node.
  /**
   * If a node_name is not provided, defaults to the current node.
   * The {parameter_name, node_name} key on the parameter_callbacks_ map will be erased, removing
   * all callback associated with that parameter.
   *
   * \param[in] parameter_name Name of parameter.
   * \param[in] node_name Name of node which hosts the parameter.
   */
  void
  remove_parameter_callback(
    const std::string & parameter_name,
    const std::string & node_name = "");

  /// Get a rclcpp::Parameter from parameter event, return true if parameter name & node in event.
  /**
   * If a node_name is not provided, defaults to the current node.
   *
   * \param[in] event Event msg to be inspected.
   * \param[out] parameter Reference to rclcpp::Parameter to be assigned.
   * \param[in] parameter_name Name of parameter.
   * \param[in] node_name Name of node which hosts the parameter.
   * \returns true if requested parameter name & node is in event, false otherwise
   */
  static bool
  get_parameter_from_event(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event,
    rclcpp::Parameter & parameter,
    const std::string parameter_name,
    const std::string node_name = "");

  /// Get a rclcpp::Parameter from parameter event
  /**
   * If a node_name is not provided, defaults to the current node.
   *
   * The user is responsible to check if the returned parameter has been properly assigned.
   * By default, if the requested parameter is not found in the event, the returned parameter
   * has parameter value of type rclcpp::PARAMETER_NOT_SET.
   *
   * \param[in] event Event msg to be inspected.
   * \param[in] parameter_name Name of parameter.
   * \param[in] node_name Name of node which hosts the parameter.
   * \returns The resultant rclcpp::Parameter from the event
   */
  static rclcpp::Parameter
  get_parameter_from_event(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event,
    const std::string parameter_name,
    const std::string node_name = "");

  using CallbacksContainerType = std::list<ParameterEventsCallbackHandle::WeakPtr>;

protected:
  /// Add a subscription (if unique) to a namespace parameter events topic.
  void
  add_namespace_event_subscriber(const std::string & node_namespace);

  /// Remove a subscription to a namespace parameter events topic.
  void
  remove_namespace_event_subscriber(const std::string & node_namespace);

  /// Return true if any callbacks still exist on a namespace event topic, otherwise return false
  bool
  should_unsubscribe_to_namespace(const std::string & node_namespace);

  /// Callback for parameter events subscriptions.
  void
  event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  // Utility functions for string and path name operations.
  std::string resolve_path(const std::string & path);
  std::pair<std::string, std::string> split_path(const std::string & str);
  std::string join_path(std::string path, std::string name);

  // Node Interfaces used for logging and creating subscribers.
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

  rclcpp::QoS qos_;

  // *INDENT-OFF* Uncrustify doesn't handle indented public/private labels
  // Hash function for string pair required in std::unordered_map
  class StringPairHash
  {
  public:
    template<typename T>
    inline void hash_combine(std::size_t & seed, const T & v) const
    {
      std::hash<T> hasher;
      seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    inline size_t operator()(const std::pair<std::string, std::string> & s) const
    {
      size_t seed = 0;
      hash_combine(seed, s.first);
      hash_combine(seed, s.second);
      return seed;
    }
  };
  // *INDENT-ON*

  // Map container for registered parameters.
  std::unordered_map<
    std::pair<std::string, std::string>,
    CallbacksContainerType,
    StringPairHash
  > parameter_callbacks_;

  // Vector of unique namespaces added.
  std::vector<std::string> subscribed_namespaces_;
  // Vector of event callback namespaces
  std::vector<std::string> event_namespaces_;

  // Vector of event subscriptions for each namespace.
  std::vector<rclcpp::Subscription
    <rcl_interfaces::msg::ParameterEvent>::SharedPtr> event_subscriptions_;

  std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr &)> event_callback_;

  std::recursive_mutex mutex_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__PARAMETER_EVENTS_SUBSCRIBER_HPP_
