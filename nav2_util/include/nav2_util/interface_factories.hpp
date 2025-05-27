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

#ifndef NAV2_UTIL__INTERFACE_FACTORIES_HPP_
#define NAV2_UTIL__INTERFACE_FACTORIES_HPP_

#include "nav2_util/qos_profiles.hpp"
#include "nav2_util/service_client.hpp"
#include "nav2_util/service_server.hpp"
#include "nav2_util/simple_action_server.hpp"

namespace nav2_qos
{

/**
 * @class nav2_qos::SubscriptionOptions
 * @brief Nav2's subscription options to extend rclcpp's configurability
 */
class SubscriptionOptions : public rclcpp::SubscriptionOptions
{
public:
  /**
   * @brief Constructor for SubscriptionOptions
   * @param allow_parameter_qos_overrides Whether to allow QoS overrides for this subscription
   * @param callback_group_ptr Pointer to the callback group to use for this subscription
   * @param qos_message_lost_callback Callback for when a QoS message is lost
   * @param subscription_matched_callback Callback when a subscription is matched with a publisher
   * @param incompatible_qos_callback Callback for when an incompatible QoS is requested
   * @param qos_requested_incompatible_qos_callback Callback for when a QoS request is incompatible
   * @param qos_deadline_requested_callback Callback for when a QoS deadline is missed
   * @param qos_liveliness_changed_callback Callback for when a QoS liveliness change occurs
   */
  explicit
  SubscriptionOptions(
    const bool allow_parameter_qos_overrides = true,
    const rclcpp::CallbackGroup::SharedPtr callback_group_ptr = nullptr,
    rclcpp::QOSMessageLostCallbackType qos_message_lost_callback = nullptr,
    rclcpp::SubscriptionMatchedCallbackType subscription_matched_callback = nullptr,
    rclcpp::IncompatibleTypeCallbackType incompatible_qos_callback = nullptr,
    rclcpp::QOSRequestedIncompatibleQoSCallbackType qos_requested_incompatible_qos_callback = nullptr,
    rclcpp::QOSDeadlineRequestedCallbackType qos_deadline_requested_callback = nullptr,
    rclcpp::QOSLivelinessChangedCallbackType qos_liveliness_changed_callback = nullptr)
  {
    // Allow for all topics to have QoS overrides
    if (allow_parameter_qos_overrides) {
      this->qos_overriding_options = rclcpp::QosOverridingOptions(
        {rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::Reliability, rclcpp::QosPolicyKind::History});
    }

    // Set the callback group to use for this subscription, if given
    this->callback_group = callback_group_ptr;

    // Set the event callbacks
    this->event_callbacks.deadline_callback = qos_deadline_requested_callback;
    this->event_callbacks.liveliness_callback = qos_liveliness_changed_callback;
    this->event_callbacks.incompatible_qos_callback =
      qos_requested_incompatible_qos_callback;
    this->event_callbacks.message_lost_callback = qos_message_lost_callback;
    this->event_callbacks.incompatible_type_callback = incompatible_qos_callback;
    this->event_callbacks.matched_callback = subscription_matched_callback;
  }
};

/**
 * @class nav2_qos::PublisherOptions
 * @brief Nav2's Publisher options to extend rclcpp's configurability
 */
class PublisherOptions : public rclcpp::PublisherOptions
{
  /**
   * @brief Constructor for PublisherOptions
   * @param allow_parameter_qos_overrides Whether to allow QoS overrides for this publisher
   * @param callback_group_ptr Pointer to the callback group to use for this subscription
   * @param publisher_matched_callback Callback when a publisher is matched with a subscriber
   * @param qos_offered_incompatible_qos_callback Callback for when a QoS request is incompatible
   * @param incompatible_qos_callback Callback for when an incompatible QoS is requested
   * @param qos_deadline_offered_callback Callback for when a QoS deadline is missed
   * @param qos_liveliness_lost_callback Callback for when a QoS liveliness change occurs
   */
  explicit
  PublisherOptions(
    const bool allow_parameter_qos_overrides = true,
    const rclcpp::CallbackGroup::SharedPtr callback_group_ptr = nullptr,
    rclcpp::PublisherMatchedCallbackType publisher_matched_callback = nullptr,
    rclcpp::IncompatibleTypeCallbackType incompatible_qos_callback = nullptr,
    rclcpp::QOSOfferedIncompatibleQoSCallbackType qos_offered_incompatible_qos_callback = nullptr,
    rclcpp::QOSDeadlineOfferedCallbackType qos_deadline_offered_callback = nullptr,
    rclcpp::QOSLivelinessLostCallbackType qos_liveliness_lost_callback = nullptr)
  {
    // Allow for all topics to have QoS overrides
    if (allow_parameter_qos_overrides) {
      this->qos_overriding_options = rclcpp::QosOverridingOptions(
        {rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::Reliability, rclcpp::QosPolicyKind::History});
    }

    // Set the callback group to use for this publisher, if given
    this->callback_group = callback_group_ptr;

    // Set the event callbacks
    this->event_callbacks.deadline_callback = qos_deadline_offered_callback;
    this->event_callbacks.liveliness_callback = qos_liveliness_lost_callback;
    this->event_callbacks.incompatible_qos_callback =
      qos_offered_incompatible_qos_callback;
    this->event_callbacks.incompatible_type_callback = incompatible_qos_callback;
    this->event_callbacks.matched_callback = publisher_matched_callback;
  }
};

/**
 * @brief Create a subscription to a topic using Nav2 QoS profiles and SubscriptionOptions
 * @param node Node to create the subscription on
 * @param topic_name Name of topic
 * @param callback Callback function to handle incoming messages
 * @param qos QoS settings for the subscription (default is nav2_qos::StandardTopicQoS())
 * @param callback_group The callback group to use (if provided)
 * @return A shared pointer to the created subscription
 */
template<typename NodeT, typename MessageT, typename CallbackT>
typename rclcpp::Subscription<MessageT>::SharedPtr create_subscription(
  const typename NodeT::SharedPtr & node,
  const std::string & topic_name,
  CallbackT && callback,
  const rclcpp::QoS & qos = nav2_qos::StandardTopicQoS(),
  const rclcpp::CallbackGroup::SharedPtr & callback_group = nullptr)
{
  bool allow_parameter_qos_overrides = nav2_util::declare_or_get_parameter<bool>(
    node, "allow_parameter_qos_overrides", true);
  return node->template create_subscription<MessageT>(
    topic_name,
    std::forward<CallbackT>(callback),
    qos,
    nav2_qos::SubscriptionOptions(allow_parameter_qos_overrides, callback_group));
}

/**
 * @brief Create a publisher to a topic using Nav2 QoS profiles and PublisherOptions
 * @param node Node to create the publisher on
 * @param topic_name Name of topic
 * @param qos QoS settings for the publisher (default is nav2_qos::StandardTopicQoS())
 * @param callback_group The callback group to use (if provided)
 * @return A shared pointer to the created publisher
 */
template<typename NodeT, typename MessageT>
typename rclcpp::Publisher<MessageT>::SharedPtr create_publisher(
  const typename NodeT::SharedPtr & node,
  const std::string & topic_name,
  const rclcpp::QoS & qos = nav2_qos::StandardTopicQoS(),
  const rclcpp::CallbackGroup::SharedPtr & callback_group = nullptr)
{
  bool allow_parameter_qos_overrides = nav2_util::declare_or_get_parameter<bool>(
    node, "allow_parameter_qos_overrides", true);
  return node->template create_publisher<MessageT>(
    topic_name,
    qos,
    nav2_qos::PublisherOptions(allow_parameter_qos_overrides, callback_group));
}

/**
 * @brief Create a ServiceClient to interface with a service
 * @param node Node to create the service client on
 * @param service_name Name of service
 * @param use_internal_executor Whether to use the internal executor (default is false)
 * @return A shared pointer to the created nav2_util::ServiceClient
 */
template<typename NodeT, typename SrvT>
std::shared_ptr<nav2_util::ServiceClient<SrvT, NodeT>> create_client(
  const typename NodeT::SharedPtr & node,
  const std::string & service_name,
  bool use_internal_executor = false)
{
  return std::make_shared<nav2_util::ServiceClient<SrvT, NodeT>>(
    service_name, node, use_internal_executor);
}

/**
 * @brief Create a ServiceServer to host with a service
 * @param node Node to create the service server on
 * @param service_name Name of service
 * @param callback Callback function to handle service requests
 * @param callback_group The callback group to use (if provided)
 * @return A shared pointer to the created nav2_util::ServiceServer
 */
template<typename NodeT, typename SrvT>
std::shared_ptr<nav2_util::ServiceServer<SrvT, NodeT>> create_service(
  const typename NodeT::SharedPtr & node,
  const std::string & service_name,
  typename nav2_util::ServiceServer<SrvT, NodeT>::CallbackType callback,
  rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
{
  return std::make_shared<nav2_util::ServiceServer<SrvT, NodeT>>(
    service_name, node, callback, callback_group);
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
 * @return A shared pointer to the created nav2_util::SimpleActionServer
 */
template<typename NodeT, typename ActionT>
std::shared_ptr<nav2_util::SimpleActionServer<ActionT>> create_server(
  const NodeT & node,
  const std::string & action_name,
  typename nav2_util::SimpleActionServer<ActionT>::ExecuteCallback execute_callback,
  typename nav2_util::SimpleActionServer<ActionT>::CompletionCallback completion_callback = nullptr,
  std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
  bool spin_thread = false,
  const bool realtime = false)
{
  return std::make_shared<nav2_util::SimpleActionServer<ActionT>>(
    node, action_name, execute_callback, completion_callback, server_timeout, spin_thread, realtime);
}

}  // namespace nav2_qos

#endif  // NAV2_UTIL__INTERFACE_FACTORIES_HPP_
