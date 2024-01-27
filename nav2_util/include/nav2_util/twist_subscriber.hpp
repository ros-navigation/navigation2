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
 * @class nav2_util::TwistSubscriber
 * @brief A simple wrapper on a Twist subscriber that receives either
 *        geometry_msgs::msg::TwistStamped or geometry_msgs::msg::Twist
 *
 * @note Usage:
 * The default behavior is to subscribe to Twist, which preserves backwards compatibility
 * with ROS distributions up to Iron.
 * The behavior can be overridden using the "enable_stamped_cmd_vel" parameter.
 * By setting that to "True", the TwistSubscriber class would subscribe to TwistStamped.
 *
 * @note Why use Twist Stamped over Twist?
 * Twist has been used widely in many ROS applications, typically for body-frame velocity control,
 * and between ROS nodes on the same computer. Many ROS interfaces are moving to using TwistStamped
 * because it is more robust for stale data protection. This protection is especially important
 * when sending velocity control over lossy communication links.
 * An example application where this matters is a drone with a Linux computer running a ROS
 * controller that sends Twist commands to an embedded autopilot. If the autopilot failed to
 * recognize a highly latent connection, it could result in instability or a crash because of the
 * decreased phase margin for control.
 * TwistStamped also has a frame ID, allowing explicit control for multiple frames, rather than
 * relying on an assumption of body-frame control or having to create a different topic.
 * Adding a header is low-cost for most ROS applications; the header can be set to an empty string
 * if bandwidth is of concern.
 *
 * @note Implementation Design Notes:
 * Compared to the naive approach of setting up one subscriber for each message type,
 * only one subscriber is created at a time; the other is nullptr.
 * This reduces RAM usage and ROS discovery traffic.
 * This approach allows NAV2 libraries to be flexible in which Twist message they support,
 * while maintaining a stable API in a ROS distribution.
 *
 */

class TwistSubscriber
{
public:
  /**
  * @brief A constructor that supports either Twist and TwistStamped
  * @param node The node to add the Twist subscriber to
  * @param topic The subscriber topic name
  * @param qos The subscriber quality of service
  * @param TwistCallback The subscriber callback for Twist messages
  * @param TwistStampedCallback The subscriber callback for TwistStamped messages
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
    nav2_util::declare_parameter_if_not_declared(
      node, "enable_stamped_cmd_vel",
      rclcpp::ParameterValue(false));
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

  /**
  * @brief A constructor that only supports TwistStamped
  * @param node The node to add the TwistStamped subscriber to
  * @param topic The subscriber topic name
  * @param qos The subscriber quality of service
  * @param TwistStampedCallback The subscriber callback for TwistStamped messages
  * @throw std::invalid_argument When configured with an invalid ROS parameter
  */
  template<typename TwistStampedCallbackT>
  explicit TwistSubscriber(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & topic,
    const rclcpp::QoS & qos,
    TwistStampedCallbackT && TwistStampedCallback
  )
  {
    nav2_util::declare_parameter_if_not_declared(
      node, "enable_stamped_cmd_vel",
      rclcpp::ParameterValue(false));
    node->get_parameter("enable_stamped_cmd_vel", is_stamped_);
    if (is_stamped_) {
      twist_stamped_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic,
        qos,
        std::forward<TwistStampedCallbackT>(TwistStampedCallback));
    } else {
      throw std::invalid_argument(
              "enable_stamped_cmd_vel must be true when using this constructor!");
    }
  }

protected:
  //! @brief The user-configured value for ROS parameter enable_stamped_cmd_vel
  bool is_stamped_{false};
  //! @brief The subscription when using Twist
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_ {nullptr};
  //! @brief The subscription when using TwistStamped
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_sub_ {nullptr};
};


}  // namespace nav2_util

#endif  // NAV2_UTIL__TWIST_SUBSCRIBER_HPP_
