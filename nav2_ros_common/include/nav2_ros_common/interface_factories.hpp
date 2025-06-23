// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_ROS_COMMON__INTERFACE_FACTORIES_HPP_
#define NAV2_ROS_COMMON__INTERFACE_FACTORIES_HPP_

#include <utility>
#include <string>
#include <memory>
#include "nav2_ros_common/qos_profiles.hpp"
#include "nav2_ros_common/service_client.hpp"
#include "nav2_ros_common/service_server.hpp"
#include "nav2_ros_common/simple_action_server.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_ros_common/publisher.hpp"
#include "nav2_ros_common/subscription.hpp"
#include "nav2_ros_common/action_client.hpp"
#include "rclcpp_action/client.hpp"

namespace nav2
{

namespace interfaces
{

/**
 * @brief Create a SubscriptionOptions object with Nav2's QoS profiles and options
 * @param allow_parameter_qos_overrides Whether to allow QoS overrides for this subscription
 * @param callback_group_ptr Pointer to the callback group to use for this subscription
 * @param qos_message_lost_callback Callback for when a QoS message is lost
 * @param subscription_matched_callback Callback when a subscription is matched with a publisher
 * @param incompatible_qos_type_callback Callback for when an incompatible QoS type is requested
 * @param qos_requested_incompatible_qos_callback Callback for when a QoS request is incompatible
 * @param qos_deadline_requested_callback Callback for when a QoS deadline is missed
 * @param qos_liveliness_changed_callback Callback for when a QoS liveliness change occurs
 * @return A nav2::SubscriptionOptions object with the specified configurations
 */
inline rclcpp::SubscriptionOptions createSubscriptionOptions(
  const bool allow_parameter_qos_overrides = true,
  const rclcpp::CallbackGroup::SharedPtr callback_group_ptr = nullptr,
  rclcpp::QOSMessageLostCallbackType qos_message_lost_callback = nullptr,
  rclcpp::SubscriptionMatchedCallbackType subscription_matched_callback = nullptr,
  rclcpp::IncompatibleTypeCallbackType incompatible_qos_type_callback = nullptr,
  rclcpp::QOSRequestedIncompatibleQoSCallbackType requested_incompatible_qos_callback = nullptr,
  rclcpp::QOSDeadlineRequestedCallbackType qos_deadline_requested_callback = nullptr,
  rclcpp::QOSLivelinessChangedCallbackType qos_liveliness_changed_callback = nullptr)
{
  rclcpp::SubscriptionOptions options;
  // Allow for all topics to have QoS overrides
  if (allow_parameter_qos_overrides) {
    options.qos_overriding_options = rclcpp::QosOverridingOptions(
      {rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::Reliability, rclcpp::QosPolicyKind::History});
  }

  // Set the callback group to use for this subscription, if given
  options.callback_group = callback_group_ptr;

  // Set the event callbacks
  options.event_callbacks.deadline_callback = qos_deadline_requested_callback;
  options.event_callbacks.liveliness_callback = qos_liveliness_changed_callback;
  options.event_callbacks.incompatible_qos_callback =
    requested_incompatible_qos_callback;
  options.event_callbacks.message_lost_callback = qos_message_lost_callback;
  options.event_callbacks.incompatible_type_callback = incompatible_qos_type_callback;
  options.event_callbacks.matched_callback = subscription_matched_callback;
  return options;
}

/**
 * @brief Create a PublisherOptions object with Nav2's QoS profiles and options
 * @param allow_parameter_qos_overrides Whether to allow QoS overrides for this publisher
 * @param callback_group_ptr Pointer to the callback group to use for this publisher
 * @param publisher_matched_callback Callback when a publisher is matched with a subscriber
 * @param incompatible_qos_type_callback Callback for when an incompatible QoS type is requested
 * @param offered_incompatible_qos_cb Callback for when a QoS request is incompatible
 * @param qos_deadline_offered_callback Callback for when a QoS deadline is missed
 * @param qos_liveliness_lost_callback Callback for when a QoS liveliness change occurs
 * @return A rclcpp::PublisherOptions object with the specified configurations
 */
inline rclcpp::PublisherOptions createPublisherOptions(
  const bool allow_parameter_qos_overrides = true,
  const rclcpp::CallbackGroup::SharedPtr callback_group_ptr = nullptr,
  rclcpp::PublisherMatchedCallbackType publisher_matched_callback = nullptr,
  rclcpp::IncompatibleTypeCallbackType incompatible_qos_type_callback = nullptr,
  rclcpp::QOSOfferedIncompatibleQoSCallbackType offered_incompatible_qos_cb = nullptr,
  rclcpp::QOSDeadlineOfferedCallbackType qos_deadline_offered_callback = nullptr,
  rclcpp::QOSLivelinessLostCallbackType qos_liveliness_lost_callback = nullptr)
{
  rclcpp::PublisherOptions options;
  // Allow for all topics to have QoS overrides
  if (allow_parameter_qos_overrides) {
    options.qos_overriding_options = rclcpp::QosOverridingOptions(
      {rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::Reliability, rclcpp::QosPolicyKind::History});
  }

  // Set the callback group to use for this publisher, if given
  options.callback_group = callback_group_ptr;

  // Set the event callbacks
  options.event_callbacks.deadline_callback = qos_deadline_offered_callback;
  options.event_callbacks.liveliness_callback = qos_liveliness_lost_callback;
  options.event_callbacks.incompatible_qos_callback = offered_incompatible_qos_cb;
  options.event_callbacks.incompatible_type_callback = incompatible_qos_type_callback;
  options.event_callbacks.matched_callback = publisher_matched_callback;
  return options;
}

/**
 * @brief Create a subscription to a topic using Nav2 QoS profiles and SubscriptionOptions
 * @param node Node to create the subscription on
 * @param topic_name Name of topic
 * @param callback Callback function to handle incoming messages
 * @param qos QoS settings for the subscription (default is nav2::qos::StandardTopicQoS())
 * @param callback_group The callback group to use (if provided)
 * @return A shared pointer to the created subscription
 */
template<typename MessageT, typename NodeT, typename CallbackT>
typename nav2::Subscription<MessageT>::SharedPtr create_subscription(
  const NodeT & node,
  const std::string & topic_name,
  CallbackT && callback,
  const rclcpp::QoS & qos = nav2::qos::StandardTopicQoS(),
  const rclcpp::CallbackGroup::SharedPtr & callback_group = nullptr)
{
  bool allow_parameter_qos_overrides = nav2::declare_or_get_parameter(
    node, "allow_parameter_qos_overrides", false);

  auto params_interface = node->get_node_parameters_interface();
  auto topics_interface = node->get_node_topics_interface();
  return rclcpp::create_subscription<MessageT, CallbackT>(
    params_interface,
    topics_interface,
    topic_name,
    qos,
    std::forward<CallbackT>(callback),
    createSubscriptionOptions(allow_parameter_qos_overrides, callback_group));
}

/**
 * @brief Create a publisher to a topic using Nav2 QoS profiles and PublisherOptions
 * @param node Node to create the publisher on
 * @param topic_name Name of topic
 * @param qos QoS settings for the publisher (default is nav2::qos::StandardTopicQoS())
 * @param callback_group The callback group to use (if provided)
 * @return A shared pointer to the created publisher
 */
template<typename MessageT, typename NodeT>
typename nav2::Publisher<MessageT>::SharedPtr create_publisher(
  const NodeT & node,
  const std::string & topic_name,
  const rclcpp::QoS & qos = nav2::qos::StandardTopicQoS(),
  const rclcpp::CallbackGroup::SharedPtr & callback_group = nullptr)
{
  bool allow_parameter_qos_overrides = nav2::declare_or_get_parameter(
    node, "allow_parameter_qos_overrides", false);
  using PublisherT = nav2::Publisher<MessageT>;
  auto pub = rclcpp::create_publisher<MessageT, std::allocator<void>, PublisherT>(
    *node,
    topic_name,
    qos,
    createPublisherOptions(allow_parameter_qos_overrides, callback_group));
  return pub;
}

/**
 * @brief Create a ServiceClient to interface with a service
 * @param node Node to create the service client on
 * @param service_name Name of service
 * @param use_internal_executor Whether to use the internal executor (default is false)
 * @return A shared pointer to the created nav2::ServiceClient
 */
template<typename SrvT, typename NodeT>
typename nav2::ServiceClient<SrvT>::SharedPtr create_client(
  const NodeT & node,
  const std::string & service_name,
  bool use_internal_executor = false)
{
  return std::make_shared<nav2::ServiceClient<SrvT>>(
    service_name, node, use_internal_executor);
}

/**
 * @brief Create a ServiceServer to host with a service
 * @param node Node to create the service server on
 * @param service_name Name of service
 * @param callback Callback function to handle service requests
 * @param callback_group The callback group to use (if provided)
 * @return A shared pointer to the created nav2::ServiceServer
 */
template<typename SrvT, typename NodeT, typename CallbackT>
typename nav2::ServiceServer<SrvT>::SharedPtr create_service(
  const NodeT & node,
  const std::string & service_name,
  CallbackT && callback,
  rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
{
  using Request = typename SrvT::Request;
  using Response = typename SrvT::Response;
  using CallbackFn = std::function<void(
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<Request>,
        std::shared_ptr<Response>)>;
  CallbackFn cb = std::forward<CallbackT>(callback);

  return std::make_shared<nav2::ServiceServer<SrvT>>(
    service_name, node, cb, callback_group);
}

/**
 * @brief Create a SimpleActionServer to host with an action
 * @param node Node to create the action server on
 * @param action_name Name of action
 * @param execute_callback Callback function to handle action execution
 * @param completion_callback Callback function to handle action completion (optional)
 * @param server_timeout Timeout for the action server (default is 500ms)
 * @param spin_thread Whether to spin with a dedicated thread internally (default is false)
 * @param realtime Whether the action server's worker thread
 * should have elevated prioritization (soft realtime)
 * @return A shared pointer to the created nav2::SimpleActionServer
 */
template<typename ActionT, typename NodeT>
typename nav2::SimpleActionServer<ActionT>::SharedPtr create_action_server(
  const NodeT & node,
  const std::string & action_name,
  typename nav2::SimpleActionServer<ActionT>::ExecuteCallback execute_callback,
  typename nav2::SimpleActionServer<ActionT>::CompletionCallback complete_cb = nullptr,
  std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
  bool spin_thread = false,
  const bool realtime = false)
{
  return std::make_shared<nav2::SimpleActionServer<ActionT>>(
    node, action_name, execute_callback, complete_cb, server_timeout, spin_thread, realtime);
}

/**
 * @brief Create an action client for a specific action type
 * @param node Node to create the action client on
 * @param action_name Name of the action
 * @param callback_group The callback group to use (if provided)
 * @return A shared pointer to the created nav2::ActionClient
 */
template<typename ActionT, typename NodeT>
typename nav2::ActionClient<ActionT>::SharedPtr create_action_client(
  const NodeT & node,
  const std::string & action_name,
  rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
{
  auto client = rclcpp_action::create_client<ActionT>(node, action_name, callback_group);
  nav2::setIntrospectionMode(client,
    node->get_node_parameters_interface(), node->get_clock());
  return client;
}

}  // namespace interfaces

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__INTERFACE_FACTORIES_HPP_
