// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_TASKS__IS_STUCK_CONDITION_HPP_
#define NAV2_TASKS__IS_STUCK_CONDITION_HPP_

#include <string>
#include <chrono>
#include <ctime>
#include <cmath>
#include <thread>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "behavior_tree_core/condition_node.h"
#include "nav2_robot/robot.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_tasks
{

class IsStuckCondition : public BT::ConditionNode, public rclcpp::Node
{
public:
  explicit IsStuckCondition(const std::string & condition_name)
  : BT::ConditionNode(condition_name),
    Node("IsStuckCondition")
  {
    RCLCPP_INFO(get_logger(), "IsStuckCondition::constructor");

    // Capture velocity commands published by other nodes
    // TODO(orduno) #381 Currently DWB is publishing commands directly, not using the Robot class.
    //              #383 Once all nodes use the Robot class we can change this as well.
    vel_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        if (current_vel_cmd_ != nullptr) {
          if (previous_vel_cmd_->linear.x != current_vel_cmd_->linear.x) {
            previous_vel_cmd_ = current_vel_cmd_;
          }
        } else {
          previous_vel_cmd_ = msg;
        }
        current_vel_cmd_ = msg;
      }
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {current_velocity_ = msg;});
  }

  IsStuckCondition() = delete;

  ~IsStuckCondition()
  {
  }

  BT::NodeStatus tick() override
  {
    if (isStuck()) {
      logMessage("tick(): Robot stuck!");
      return BT::NodeStatus::SUCCESS;
    }

    logMessage("tick(): Robot not stuck");
    return BT::NodeStatus::FAILURE;
  }

  bool isStuck()
  {
    // TODO(orduno) #383 Move algorithm to the robot class
    // return robot_.isStuck();

    // Spin the node to get messages from the subscriptions
    rclcpp::spin_some(this->get_node_base_interface());

    if (current_velocity_ == nullptr) {
      RCLCPP_WARN_ONCE(get_logger(), "Initial odometry not yet received.");
      return false;
    }

    if (current_vel_cmd_ == nullptr) {
      RCLCPP_WARN_ONCE(get_logger(), "Velocity commands have not been published.");
      return false;
    }

    // TODO(orduno) #400 The robot getting stuck can result on different types of motion
    // depending on the state prior to getting stuck (sudden change in accel, not moving at all,
    // random oscillations, etc). For now, we only address the case where the commanded velocity
    // is non-zero but the robot is not accelerating. A better approach is to do a forward
    // simulation of the robot motion and compare it with the actual one.

    // Noise in the odom measurements observed in simulation with Gazebo with Turtlebot3
    double odom_linear_vel_error = 0.0005;

    // TODO(orduno) assuming the robot is moving forward

    double v1 = current_velocity_->twist.twist.linear.x;
    std::this_thread::sleep_for(1s);
    rclcpp::spin_some(this->get_node_base_interface());
    double v2 = current_velocity_->twist.twist.linear.x;

    if (std::abs(current_vel_cmd_->linear.x) > odom_linear_vel_error) {
      // Commanded velocity is non-zero

      // TODO(orduno) Check if there was a change in traveling direction

      // Only considering forward velocity
      double curr_cmd = current_vel_cmd_->linear.x;
      double prev_cmd = previous_vel_cmd_->linear.x;

      // Assuming smooth velocity commands
      // TODO(orduno) Address case where the robot is moving backwards

      // Allow velocity fluctuations up to 20% of vel command
      double tolerance = curr_cmd * 0.2;

      if (curr_cmd >= prev_cmd) {
        // Robot should be accelerating, it's ok if velocity overshoots command
        if ((v2 + tolerance + odom_linear_vel_error) < v1) {
          RCLCPP_WARN(get_logger(),
            "The robot is not accelerating, previous cmd: %.6f, current cmd: %.6f,"
            "  v1: %.6f, v2: %.6f", prev_cmd, curr_cmd, v1, v2);
          return true;
        }
      } else {
        // // Robot should be decelerating
        // if ((v2 - tolerance - odom_linear_vel_error) > v1) {
        //   RCLCPP_WARN(get_logger(),
        //     "The robot is not decelerating, previous cmd: %.6f, current cmd: %.6f,"
        //     "  v1: %.6f, v2: %.6f", prev_cmd, curr_cmd, v1, v2);
        //   return true;
        // }
      }
    }

    return false;
  }

  void logMessage(const std::string & msg) const
  {
    static std::string prev_msg;

    if (msg == prev_msg) {
      return;
    }

    RCLCPP_INFO(get_logger(), msg);
    prev_msg = msg;
  }

  void halt() override
  {
  }

private:
  // Listen to odometry
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // The current velocity as received from the Odometry subscription
  std::shared_ptr<nav_msgs::msg::Odometry> current_velocity_;

  // Listen to the controller publishing velocity commands
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;
  // The last velocity command published by the controller
  std::shared_ptr<geometry_msgs::msg::Twist> current_vel_cmd_;
  std::shared_ptr<geometry_msgs::msg::Twist> previous_vel_cmd_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__IS_STUCK_CONDITION_HPP_
