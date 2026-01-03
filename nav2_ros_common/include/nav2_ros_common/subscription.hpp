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

#ifndef NAV2_ROS_COMMON__SUBSCRIPTION_HPP_
#define NAV2_ROS_COMMON__SUBSCRIPTION_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/managed_entity.hpp"

namespace nav2
{

/**
 * @class nav2::Subscription
 * @brief A lifecycle-enabled ROS 2 subscription for Nav2
 *
 * - Implements lifecycle transitions via SimpleManagedEntity
 * - Gates user callbacks using is_activated()
 * - For NON-lifecycle nodes (plain rclcpp::Node), it auto-activates so behavior matches old code.
 */
template<typename MessageT>
class Subscription : public rclcpp_lifecycle::SimpleManagedEntity
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)

  template<typename NodeT, typename CallbackT>
  Subscription(
    const NodeT & node,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    CallbackT && user_callback,
    const rclcpp::SubscriptionOptions & options = rclcpp::SubscriptionOptions{})
  : topic_name_(topic_name),
    logger_(node->get_logger()),
    should_log_(true)
  {
    auto wrapped_cb = make_wrapped_callback(std::forward<CallbackT>(user_callback));

    sub_ = node->template create_subscription<MessageT>(
      topic_name_, qos, std::move(wrapped_cb), options);

    // If this is NOT a lifecycle node, there will be no lifecycle manager to call on_activate().
    // To keep old behavior (callbacks run immediately), we auto-activate.
    auto maybe_lc = std::dynamic_pointer_cast<rclcpp_lifecycle::LifecycleNode>(node);
    if (!maybe_lc) {
      this->on_activate();
    }
  }

  void on_activate() override
  {
    rclcpp_lifecycle::SimpleManagedEntity::on_activate();
    // No need to touch should_log_ here (logging happens only when inactive).
  }

  void on_deactivate() override
  {
    rclcpp_lifecycle::SimpleManagedEntity::on_deactivate();
    // New inactive period -> allow one warning again
    should_log_.store(true);
  }

private:
  template<typename CallbackU>
  auto make_wrapped_callback(CallbackU && user_callback)
  {
    using CB = std::decay_t<CallbackU>;
    using UniquePtr = typename MessageT::UniquePtr;
    using SharedPtr = typename MessageT::SharedPtr;
    using ConstSharedPtr = typename MessageT::ConstSharedPtr;
    using MsgInfo = const rclcpp::MessageInfo &;

    // Use shared_ptr so the lambda stays COPIABLE (rclcpp often requires copiable callbacks)
    auto cb_ptr = std::make_shared<CB>(std::forward<CallbackU>(user_callback));

    if constexpr (std::is_invocable_v<CB &, UniquePtr>) {
      return [this, cb_ptr](UniquePtr msg) {
               if (!this->is_activated()) {
                 log_subscription_not_enabled_once();
                 return;
               }
               should_log_.store(true);
               (*cb_ptr)(std::move(msg));
             };
    } else if constexpr (std::is_invocable_v<CB &, SharedPtr>) {
      return [this, cb_ptr](SharedPtr msg) {
               if (!this->is_activated()) {
                 log_subscription_not_enabled_once();
                 return;
               }
               should_log_.store(true);
               (*cb_ptr)(std::move(msg));
             };
    } else if constexpr (std::is_invocable_v<CB &, ConstSharedPtr>) {
      return [this, cb_ptr](ConstSharedPtr msg) {
               if (!this->is_activated()) {
                 log_subscription_not_enabled_once();
                 return;
               }
               should_log_.store(true);
               (*cb_ptr)(std::move(msg));
             };
    } else if constexpr (std::is_invocable_v<CB &, UniquePtr, MsgInfo>) {
      return [this, cb_ptr](UniquePtr msg, MsgInfo info) {
               if (!this->is_activated()) {
                 log_subscription_not_enabled_once();
                 return;
               }
               should_log_.store(true);
               (*cb_ptr)(std::move(msg), info);
             };
    } else if constexpr (std::is_invocable_v<CB &, SharedPtr, MsgInfo>) {
      return [this, cb_ptr](SharedPtr msg, MsgInfo info) {
               if (!this->is_activated()) {
                 log_subscription_not_enabled_once();
                 return;
               }
               should_log_.store(true);
               (*cb_ptr)(std::move(msg), info);
             };
    } else {
      static_assert(
        sizeof(CB) == 0,
        "nav2::Subscription: unsupported callback signature for this MessageT");
    }
  }

  void log_subscription_not_enabled_once()
  {
    if (should_log_.exchange(false)) {
      RCLCPP_WARN(
        logger_,
        "Subscription on [%s] is not enabled yet (node not activated). "
        "Dropping messages until activation.",
        topic_name_.c_str());
    }
  }


  typename rclcpp::Subscription<MessageT>::SharedPtr sub_;
  std::string topic_name_;
  rclcpp::Logger logger_;
  std::atomic<bool> should_log_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__SUBSCRIPTION_HPP_
