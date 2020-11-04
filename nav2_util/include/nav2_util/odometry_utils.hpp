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
   * @param nh NodeHandle for creating subscriber
   * @param filter_duration Duration for odom history (seconds)
   * @param odom_topic Topic on which odometry should be received
   */
  explicit OdomSmoother(
    rclcpp::Node::SharedPtr nh,
    double filter_duration = 0.3,
    std::string odom_topic = "odom");

  inline geometry_msgs::msg::Twist getTwist() {return vel_smooth_.twist;}
  inline geometry_msgs::msg::TwistStamped getTwistStamped() {return vel_smooth_;}

protected:
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);
  void updateState();

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry odom_cumulate_;
  geometry_msgs::msg::TwistStamped vel_smooth_;
  std::mutex odom_mutex_;

  rclcpp::Duration odom_history_duration_;
  std::deque<nav_msgs::msg::Odometry> odom_history_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__ODOMETRY_UTILS_HPP_
