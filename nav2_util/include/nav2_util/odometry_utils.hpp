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
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace nav2_util
{

/**
 * @class OdomSmoother
 * Wrapper for getting smooth odometry readings using a simple moving average.
 * Subscribes to the topic with a mutex.
 */
class OdomSmoother
{
public:
  /**
   * @brief Overloadded Constructor for nav_util::LifecycleNode parent
   * that subscribes to an Odometry topic
   * @param parent NodeHandle for creating subscriber
   * @param filter_duration Duration for odom history (seconds)
   * @param odom_topic Topic on which odometry should be received
   */
  template<typename NodeT>
  explicit OdomSmoother(
    const NodeT & parent,
    double filter_duration = 0.3,
    const std::string & odom_topic = "odom")
  : odom_history_duration_(rclcpp::Duration::from_seconds(filter_duration))
  {
    // Could be using a user rclcpp::Node, so need to use the Nav2 factory to create the
    // subscription to convert nav2::LifecycleNode, rclcpp::Node or rclcpp_lifecycle::LifecycleNode
    odom_sub_ = nav2::interfaces::create_subscription<nav_msgs::msg::Odometry>(
      parent, odom_topic,
      std::bind(&OdomSmoother::odomCallback, this, std::placeholders::_1));

    odom_cumulate_.twist.twist.linear.x = 0;
    odom_cumulate_.twist.twist.linear.y = 0;
    odom_cumulate_.twist.twist.linear.z = 0;
    odom_cumulate_.twist.twist.angular.x = 0;
    odom_cumulate_.twist.twist.angular.y = 0;
    odom_cumulate_.twist.twist.angular.z = 0;
  }

  /**
   * @brief Get twist msg from smoother
   * @return twist Twist msg
   */
  inline geometry_msgs::msg::Twist getTwist()
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!received_odom_) {
      RCLCPP_ERROR(rclcpp::get_logger("OdomSmoother"),
          "OdomSmoother has not received any data yet, returning empty Twist");
      geometry_msgs::msg::Twist twist;
      return twist;
    }
    return vel_smooth_.twist;
  }

  /**
   * @brief Get twist stamped msg from smoother
   * @return twist TwistStamped msg
   */
  inline geometry_msgs::msg::TwistStamped getTwistStamped()
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!received_odom_) {
      RCLCPP_ERROR(rclcpp::get_logger("OdomSmoother"),
          "OdomSmoother has not received any data yet, returning empty Twist");
      geometry_msgs::msg::TwistStamped twist_stamped;
      return twist_stamped;
    }
    return vel_smooth_;
  }

  /**
   * @brief Get raw twist msg from smoother (without smoothing)
   * @return twist Twist msg
   */
  inline geometry_msgs::msg::Twist getRawTwist()
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!received_odom_) {
      RCLCPP_ERROR(rclcpp::get_logger("OdomSmoother"),
          "OdomSmoother has not received any data yet, returning empty Twist");
      geometry_msgs::msg::Twist twist;
      return twist;
    }
    return odom_history_.back().twist.twist;
  }

    /**
   * @brief Get raw twist stamped msg from smoother (without smoothing)
   * @return twist TwistStamped msg
   */
  inline geometry_msgs::msg::TwistStamped getRawTwistStamped()
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    geometry_msgs::msg::TwistStamped twist_stamped;
    if (!received_odom_) {
      RCLCPP_ERROR(rclcpp::get_logger("OdomSmoother"),
          "OdomSmoother has not received any data yet, returning empty Twist");
      return twist_stamped;
    }
    twist_stamped.header = odom_history_.back().header;
    twist_stamped.twist = odom_history_.back().twist.twist;
    return twist_stamped;
  }

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

  bool received_odom_{false};
  nav2::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry odom_cumulate_;
  geometry_msgs::msg::TwistStamped vel_smooth_;
  std::mutex odom_mutex_;

  rclcpp::Duration odom_history_duration_;
  std::deque<nav_msgs::msg::Odometry> odom_history_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__ODOMETRY_UTILS_HPP_
