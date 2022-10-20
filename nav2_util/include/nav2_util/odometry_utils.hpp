// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_UTIL__ODOMETRY_UTILS_HPP_
#define NAV2_UTIL__ODOMETRY_UTILS_HPP_

#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <deque>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_util
{

/**
 * @class OdomSmoother
 * Wrapper for getting smooth odometry readings using a simple moving avergae.
 * Subscribes to the topic with a mutex.
 */
class OdomSmoother
{
public:
  /**
   * @brief Constructor that subscribes to an Odometry topic
   * @param parent NodeHandle for creating subscriber
   * @param filter_duration Duration for odom history (seconds)
   * @param odom_topic Topic on which odometry should be received
   */
  explicit OdomSmoother(
    const rclcpp::Node::WeakPtr & parent,
    double filter_duration = 0.3,
    const std::string & odom_topic = "odom");

  /**
   * @brief Overloadded Constructor for nav_util::LifecycleNode parent
   * that subscribes to an Odometry topic
   * @param parent NodeHandle for creating subscriber
   * @param filter_duration Duration for odom history (seconds)
   * @param odom_topic Topic on which odometry should be received
   */
  explicit OdomSmoother(
    const nav2_util::LifecycleNode::WeakPtr & parent,
    double filter_duration = 0.3,
    const std::string & odom_topic = "odom");

  /**
   * @brief Get twist msg from smoother
   * @return twist Twist msg
   */
  inline geometry_msgs::msg::Twist getTwist() {return vel_smooth_.twist;}

  /**
   * @brief Get twist stamped msg from smoother
   * @return twist TwistStamped msg
   */
  inline geometry_msgs::msg::TwistStamped getTwistStamped() {return vel_smooth_;}

protected:
  /**
   * @brief Callback of odometry subscriber to process
   * @param msg Odometry msg to smooth
   */
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Update internal state of the smoother after getting new data
   */
  void updateState();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry odom_cumulate_;
  geometry_msgs::msg::TwistStamped vel_smooth_;
  std::mutex odom_mutex_;

  rclcpp::Duration odom_history_duration_;
  std::deque<nav_msgs::msg::Odometry> odom_history_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__ODOMETRY_UTILS_HPP_
