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

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/get_message_type_support_handle.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rclcpp/topic_statistics/subscription_topic_statistics.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/managed_entity.hpp"

#include "nav2_ros_common/qos_profiles.hpp"

namespace nav2
{

/// Subscription that gates message handling by an activation predicate.
/// Used as the actual rclcpp subscription held by nav2::Subscription; allows
/// overriding handle_message to check activation without wrapping the user callback.
template<typename MessageT, typename Alloc = std::allocator<void>>
class LifecycleSubscription : public rclcpp::Subscription<MessageT, Alloc>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LifecycleSubscription)

  using Base = rclcpp::Subscription<MessageT, Alloc>;

  LifecycleSubscription(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const rosidl_message_type_support_t & type_support_handle,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    rclcpp::AnySubscriptionCallback<MessageT, Alloc> callback,
    const rclcpp::SubscriptionOptionsWithAllocator<Alloc> & options,
    typename Base::MessageMemoryStrategyType::SharedPtr message_memory_strategy,
    std::shared_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics>
    subscription_topic_statistics = nullptr)
  : Base(
      node_base,
      type_support_handle,
      topic_name,
      qos,
      callback,
      options,
      message_memory_strategy,
      subscription_topic_statistics)
  {}

  void set_activation_predicate(std::function<bool()> predicate)
  {
    is_activated_ = std::move(predicate);
  }

  void handle_message(
    std::shared_ptr<void> & message,
    const rclcpp::MessageInfo & message_info) override
  {
    if (!is_activated_()) {
      return;
    }
    Base::handle_message(message, message_info);
  }

  void handle_serialized_message(
    const std::shared_ptr<rclcpp::SerializedMessage> & serialized_message,
    const rclcpp::MessageInfo & message_info) override
  {
    if (!is_activated_()) {
      return;
    }
    Base::handle_serialized_message(serialized_message, message_info);
  }

  void handle_loaned_message(
    void * loaned_message,
    const rclcpp::MessageInfo & message_info) override
  {
    if (!is_activated_()) {
      return;
    }
    Base::handle_loaned_message(loaned_message, message_info);
  }

private:
  std::function<bool()> is_activated_{[]() {return false;}};
};

template<typename MessageT, typename Alloc = std::allocator<void>>
class Subscription : public rclcpp_lifecycle::SimpleManagedEntity
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)

  using LifecycleSub = LifecycleSubscription<MessageT, Alloc>;
  using Options = rclcpp::SubscriptionOptionsWithAllocator<Alloc>;

  template<typename NodeT, typename CallbackT>
  Subscription(
    const std::shared_ptr<NodeT> & node,
    const std::string & topic_name,
    CallbackT && user_callback,
    const rclcpp::QoS & qos = nav2::qos::StandardTopicQoS(),
    const Options & options = Options{})
  : topic_name_(topic_name)
  {
    init(node, qos, std::forward<CallbackT>(user_callback), options);
  }

  void on_activate() override
  {
    rclcpp_lifecycle::SimpleManagedEntity::on_activate();
  }

  void on_deactivate() override
  {
    rclcpp_lifecycle::SimpleManagedEntity::on_deactivate();
  }

  const char * get_topic_name() const noexcept
  {
    return topic_name_.c_str();
  }

protected:
  template<typename NodeT, typename CallbackT>
  void init(
    const std::shared_ptr<NodeT> & node,
    const rclcpp::QoS & qos,
    CallbackT && user_callback,
    const Options & options)
  {
    rclcpp::AnySubscriptionCallback<MessageT, Alloc> any_cb(*options.get_allocator());
    any_cb.set(std::forward<CallbackT>(user_callback));

    if (qos.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
      rclcpp_lifecycle::SimpleManagedEntity::on_activate();
    }

    auto topics_if = node->get_node_topics_interface();
    auto msg_mem_strat =
      rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::create_default();

    rclcpp::SubscriptionFactory factory{
      [options, msg_mem_strat, any_cb](
        rclcpp::node_interfaces::NodeBaseInterface * node_base_ptr,
        const std::string & topic,
        const rclcpp::QoS & actual_qos) -> rclcpp::SubscriptionBase::SharedPtr {
        auto sub = LifecycleSub::make_shared(
          node_base_ptr,
          rclcpp::get_message_type_support_handle<MessageT>(),
          topic,
          actual_qos,
          any_cb,
          options,
          msg_mem_strat,
          nullptr);
        sub->post_init_setup(node_base_ptr, actual_qos, options);
        return std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(sub);
      }
    };

    auto sub_base = topics_if->create_subscription(topic_name_, factory, qos);
    topics_if->add_subscription(sub_base, options.callback_group);

    sub_ = std::dynamic_pointer_cast<LifecycleSub>(sub_base);
    if (sub_) {
      sub_->set_activation_predicate([this]() {return is_activated();});
    }
  }

  typename LifecycleSub::SharedPtr sub_;
  std::string topic_name_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__SUBSCRIPTION_HPP_
