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

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/get_message_type_support_handle.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rclcpp_lifecycle/managed_entity.hpp"

#include "nav2_ros_common/qos_profiles.hpp"

namespace nav2
{

/// Lifecycle-managed subscription wrapper.
///
/// The underlying rclcpp::Subscription is not constructed until on_activate()
/// is called, and is destroyed on on_deactivate(). This means the topic
/// endpoint is not discoverable on the ROS graph and no callbacks fire
/// until the wrapper is activated.
template<typename MessageT, typename Alloc = std::allocator<void>>
class Subscription : public rclcpp_lifecycle::SimpleManagedEntity
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)

  using RclcppSub = rclcpp::Subscription<MessageT, Alloc>;
  using Options = rclcpp::SubscriptionOptionsWithAllocator<Alloc>;
  using AnyCallback = rclcpp::AnySubscriptionCallback<MessageT, Alloc>;

  template<typename NodeT, typename CallbackT>
  Subscription(
    const std::shared_ptr<NodeT> & node,
    const std::string & topic_name,
    CallbackT && user_callback,
    const rclcpp::QoS & qos = nav2::qos::StandardTopicQoS(),
    const Options & options = Options{})
  : topic_name_(topic_name),
    qos_(qos),
    options_(options),
    any_callback_(*options.get_allocator()),
    topics_interface_(node->get_node_topics_interface())
  {
    any_callback_.set(std::forward<CallbackT>(user_callback));
  }

  void on_activate() override
  {
    rclcpp_lifecycle::SimpleManagedEntity::on_activate();
    if (sub_) {
      return;
    }
    auto topics_if = topics_interface_.lock();
    if (!topics_if) {
      throw std::runtime_error(
              "nav2::Subscription: node topics interface expired before activation");
    }

    auto msg_mem_strat =
      rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::create_default();
    auto options = options_;
    auto callback = any_callback_;
    rclcpp::SubscriptionFactory factory{
      [options, msg_mem_strat, callback](
        rclcpp::node_interfaces::NodeBaseInterface * node_base_ptr,
        const std::string & topic,
        const rclcpp::QoS & actual_qos) -> rclcpp::SubscriptionBase::SharedPtr {
        auto sub = RclcppSub::make_shared(
          node_base_ptr,
          rclcpp::get_message_type_support_handle<MessageT>(),
          topic,
          actual_qos,
          callback,
          options,
          msg_mem_strat,
          nullptr);
        sub->post_init_setup(node_base_ptr, actual_qos, options);
        return std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(sub);
      }
    };

    auto sub_base = topics_if->create_subscription(topic_name_, factory, qos_);
    topics_if->add_subscription(sub_base, options_.callback_group);
    sub_ = std::dynamic_pointer_cast<RclcppSub>(sub_base);
  }

  void on_deactivate() override
  {
    rclcpp_lifecycle::SimpleManagedEntity::on_deactivate();
    sub_.reset();
  }

  const char * get_topic_name() const noexcept
  {
    return topic_name_.c_str();
  }

protected:
  std::string topic_name_;
  rclcpp::QoS qos_;
  Options options_;
  AnyCallback any_callback_;
  std::weak_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics_interface_;
  typename RclcppSub::SharedPtr sub_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__SUBSCRIPTION_HPP_
