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

#ifndef NAV2_ROS_COMMON__LIFECYCLE_SUBSCRIPTION_HPP_
#define NAV2_ROS_COMMON__LIFECYCLE_SUBSCRIPTION_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/managed_entity.hpp"

namespace nav2
{

/// @brief Lifecycle-aware subscription wrapper for Nav2
/**
 * This wrapper adds lifecycle management to ROS 2 subscriptions.
 * When inactive, subscription callbacks are blocked and a warning is logged.
 */
template<typename MessageT>
class LifecycleSubscription : public rclcpp_lifecycle::SimpleManagedEntity
{
public:
  using SharedPtr = std::shared_ptr<LifecycleSubscription<MessageT>>;
  using CallbackType = std::function<void(std::shared_ptr<MessageT>)>;

  /**
   * @brief Constructor
   * @param subscription The underlying ROS 2 subscription (can be nullptr initially)
   * @param topic_name Name of the topic (for logging)
   */
  LifecycleSubscription(
    typename rclcpp::Subscription<MessageT>::SharedPtr subscription,
    const std::string & topic_name)
  : subscription_(subscription),
    topic_name_(topic_name),
    should_log_(true),
    logger_(rclcpp::get_logger("LifecycleSubscription"))
  {
  }

  /**
   * @brief Set the underlying subscription (used by factory)
   */
  void set_subscription(typename rclcpp::Subscription<MessageT>::SharedPtr subscription)
  {
    subscription_ = subscription;
  }

  ~LifecycleSubscription() = default;

  /**
   * @brief Get the underlying subscription
   */
  typename rclcpp::Subscription<MessageT>::SharedPtr get_subscription() const
  {
    return subscription_;
  }

  /**
   * @brief Get the topic name
   */
  const std::string & get_topic_name() const
  {
    return topic_name_;
  }

  /**
   * @brief Activation callback - resets logging flag
   */
  void on_activate() override
  {
    SimpleManagedEntity::on_activate();
    should_log_ = true;
  }

  /**
   * @brief Check if message should be processed
   * @return true if activated, false otherwise (and logs warning)
   */
  bool should_process_message()
  {
    if (!this->is_activated()) {
      log_subscription_not_enabled();
      return false;
    }
    return true;
  }

private:
  /**
   * @brief Log that subscription is not enabled
   */
  void log_subscription_not_enabled()
  {
    if (!should_log_) {
      return;
    }

    RCLCPP_WARN(
      logger_,
      "Trying to process message on subscription '%s', but the subscription is not activated",
      topic_name_.c_str());

    should_log_ = false;
  }

  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
  std::string topic_name_;
  bool should_log_;
  rclcpp::Logger logger_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__LIFECYCLE_SUBSCRIPTION_HPP_