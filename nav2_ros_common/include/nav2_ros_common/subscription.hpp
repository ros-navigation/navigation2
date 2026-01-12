// Copyright (c) 2026 Open Navigation LLC
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
#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/managed_entity.hpp"

#include "nav2_ros_common/qos_profiles.hpp"

namespace nav2
{

template<typename MessageT, typename Alloc = std::allocator<void>>
class Subscription : public rclcpp_lifecycle::SimpleManagedEntity
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)

  using RclcppSub = rclcpp::Subscription<MessageT, Alloc>;
  using Options = rclcpp::SubscriptionOptionsWithAllocator<Alloc>;
  using AnyCb = rclcpp::AnySubscriptionCallback<MessageT, Alloc>;

  template<typename NodeT, typename CallbackT>
  Subscription(
    const std::shared_ptr<NodeT> & node,
    const std::string & topic_name,
    CallbackT && user_callback,
    const rclcpp::QoS & qos = nav2::qos::StandardTopicQoS(),
    const Options & options = Options{})
  : topic_name_(topic_name),
    logger_(node->get_logger()),
    should_log_(true),
    any_cb_(*options.get_allocator())
  {
    init(node, qos, std::forward<CallbackT>(user_callback), options);
  }

  void on_activate() override
  {
    rclcpp_lifecycle::SimpleManagedEntity::on_activate();
    should_log_.store(true);
  }

  void on_deactivate() override
  {
    rclcpp_lifecycle::SimpleManagedEntity::on_deactivate();
    should_log_.store(true);
  }

  const char * get_topic_name() const noexcept
  {
    return topic_name_.c_str();
  }

private:
  template<typename NodeT, typename CallbackT>
  void init(
    const std::shared_ptr<NodeT> & node,
    const rclcpp::QoS & qos,
    CallbackT && user_callback,
    const Options & options)
  {
    any_cb_.set(std::forward<CallbackT>(user_callback));

    auto wrapped_cb =
      [this](typename MessageT::ConstSharedPtr msg, const rclcpp::MessageInfo & info)
      {
        if (!this->is_activated()) {
          log_subscription_not_enabled_once();
          return;
        }
        should_log_.store(true);
        any_cb_.dispatch_intra_process(msg, info);
      };

    auto params_if = node->get_node_parameters_interface();
    auto topics_if = node->get_node_topics_interface();

    sub_ = rclcpp::create_subscription<MessageT>(
      params_if,
      topics_if,
      topic_name_,
      qos,
      std::move(wrapped_cb),
      options,
      rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::create_default());

    // Legacy behavior: if this is NOT a lifecycle node, auto-activate immediately.
    auto maybe_lc = std::dynamic_pointer_cast<rclcpp_lifecycle::LifecycleNode>(node);
    if (!maybe_lc) {
      this->on_activate();
    }
  }

  void log_subscription_not_enabled_once()
  {
    if (!should_log_.exchange(false)) {
      return;
    }

    RCLCPP_WARN(
      logger_,
      "Trying to take messages on topic '%s', but the subscription is not activated. "
      "Dropping until activation.",
      topic_name_.c_str());
  }

  typename RclcppSub::SharedPtr sub_;
  std::string topic_name_;
  rclcpp::Logger logger_;
  std::atomic<bool> should_log_;
  AnyCb any_cb_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__SUBSCRIPTION_HPP_
