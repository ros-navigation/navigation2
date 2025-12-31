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

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/managed_entity.hpp"

namespace nav2
{

template<typename MessageT, typename Alloc = std::allocator<void>>
class Subscription : public rclcpp_lifecycle::SimpleManagedEntity
{
public:
  using SharedPtr = std::shared_ptr<Subscription<MessageT, Alloc>>;
  using CallbackType = std::function<void(std::shared_ptr<MessageT>)>;

  explicit Subscription(
    typename rclcpp::Subscription<MessageT, Alloc>::SharedPtr subscription,
    const std::string topic_name)
  : subscription_(subscription),
    topic_name_(topic_name),
    should_log_(true),
    logger_(rclcpp::get_logger("nav2_subscription"))
  {
  }

  virtual ~Subscription() = default;

  typename rclcpp::Subscription<MessageT, Alloc>::SharedPtr get_subscription() const
  {
    return subscription_;
  }

  void set_subscription(typename rclcpp::Subscription<MessageT, Alloc>::SharedPtr subscription)
  {
    subscription_ = subscription;
  }

  const std::string & get_topic_name() const
  {
    return topic_name_;
  }

  void on_activate() override
  {
    SimpleManagedEntity::on_activate();
    should_log_ = true;
  }

  bool should_process_message()
  {
    if (!this->is_activated()) {
      log_subscription_not_enabled();
      return false;
    }
    return true;
  }

private:
  void log_subscription_not_enabled()
  {
    if (!should_log_) {
      return;
    }
    // Log the Message
    RCLCPP_WARN(
      logger_,
      "Trying to process message on subscription '%s', but  subscription  not activated",
      topic_name_.c_str());

    // We stop logging until the flag gets enabled again
    should_log_ = false;
  }

  typename rclcpp::Subscription<MessageT, Alloc>::SharedPtr subscription_;
  std::string topic_name_;
  bool should_log_ = true;
  rclcpp::Logger logger_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__SUBSCRIPTION_HPP_
