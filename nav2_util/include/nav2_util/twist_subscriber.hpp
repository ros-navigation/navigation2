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


#ifndef NAV2_UTIL__TWIST_SUBSCRIBER_HPP_
#define NAV2_UTIL__TWIST_SUBSCRIBER_HPP_


#include <memory>
#include <string>

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
 * @class nav2_util::TwistSubscriber
 * @brief A simple wrapper on a Twist subscriber that receives either Twist or TwistStamped
 *
 * The default is to subscribe to Twist to preserve backwards compatibility, but it can be overridden
 * using the "enable_stamped_cmd_vel" parameter to subscribe to TwistStamped.
 *
 */

class TwistSubscriber
{
public:
  /**
  * @brief A constructor
  * @param nh The node
  * @param topic publisher topic name
  * @param qos publisher quality of service
  */
  template<typename TwistCallbackT,
    typename TwistStampedCallbackT
  >
  explicit TwistSubscriber(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & topic,
    const rclcpp::QoS & qos,
    TwistCallbackT && TwistCallback,
    TwistStampedCallbackT && TwistStampedCallback
  )
  {
    using nav2_util::declare_parameter_if_not_declared;
    declare_parameter_if_not_declared(
      node, "enable_stamped_cmd_vel",
      rclcpp::ParameterValue{false});
    node->get_parameter("enable_stamped_cmd_vel", is_stamped_);
    if (is_stamped_) {
      twist_stamped_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic,
        qos,
        std::forward<TwistStampedCallbackT>(TwistStampedCallback));
    } else {
      twist_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        topic,
        qos,
        std::forward<TwistCallbackT>(TwistCallback));
    }
  }

protected:
  bool is_stamped_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_sub_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__TWIST_SUBSCRIBER_HPP_
