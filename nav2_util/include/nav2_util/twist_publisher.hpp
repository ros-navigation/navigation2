// Copyright (C) 2023 Ryan Friedman
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


#ifndef NAV2_UTIL__TWIST_PUBLISHER_HPP_
#define NAV2_UTIL__TWIST_PUBLISHER_HPP_


#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "lifecycle_node.hpp"
#include "node_utils.hpp"

namespace nav2_util
{

/**
 * @class nav2_util::TwistPublisher
 * @brief A simple wrapper on a Twist publisher that provides either Twist or TwistStamped
 *
 * The default is to publish Twist to preserve backwards compatibility, but it can be overridden
 * using the "enable_stamped_cmd_vel" parameter to publish TwistStamped.
 *
 */

class TwistPublisher
{
public:
  /**
  * @brief A constructor
  * @param nh The node
  * @param topic publisher topic name
  * @param qos publisher quality of service
  */
  explicit TwistPublisher(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & topic,
    const rclcpp::QoS & qos)
  : topic_(topic)
  {
    using nav2_util::declare_parameter_if_not_declared;
    declare_parameter_if_not_declared(
      node, "enable_stamped_cmd_vel",
      rclcpp::ParameterValue{false});
    node->get_parameter("enable_stamped_cmd_vel", is_stamped_);
    if (is_stamped_) {
      twist_stamped_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(
        topic_,
        qos);
    } else {
      twist_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
        topic_,
        qos);
    }
  }

  void on_activate()
  {
    if (is_stamped_) {
      twist_stamped_pub_->on_activate();
    } else {
      twist_pub_->on_activate();
    }
  }

  void on_deactivate()
  {
    if (is_stamped_) {
      twist_stamped_pub_->on_deactivate();
    } else {
      twist_pub_->on_deactivate();
    }
  }

  [[nodiscard]] bool is_activated() const
  {
    if (is_stamped_) {
      return twist_stamped_pub_->is_activated();
    } else {
      return twist_pub_->is_activated();
    }
  }

  void publish(std::unique_ptr<geometry_msgs::msg::TwistStamped> velocity)
  {
    if (is_stamped_) {
      twist_stamped_pub_->publish(std::move(velocity));
    } else {
      auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>(velocity->twist);
      twist_pub_->publish(std::move(twist_msg));
    }
  }

  [[nodiscard]] size_t get_subscription_count() const
  {
    if (is_stamped_) {
      return twist_stamped_pub_->get_subscription_count();
    } else {
      return twist_pub_->get_subscription_count();
    }
  }

protected:
  std::string topic_;
  bool is_stamped_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    twist_stamped_pub_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__TWIST_PUBLISHER_HPP_
