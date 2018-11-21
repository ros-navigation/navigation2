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

#ifndef NAV2_TASKS__IS_STUCK_CONDITION_HPP
#define NAV2_TASKS__IS_STUCK_CONDITION_HPP

#include <string>
#include <chrono>
#include <ctime>
#include <cmath>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "behavior_tree_core/condition_node.h"
#include "nav2_robot/robot.hpp"

namespace nav2_tasks
{

class IsStuckCondition : public BT::ConditionNode, public rclcpp::Node
{

public:
  explicit IsStuckCondition(const std::string & condition_name)
  : BT::ConditionNode(condition_name),
    Node("IsStuckCondition"),
    is_stuck_(false)
  {
    RCLCPP_INFO(get_logger(), "IsStuckCondition::constructor");

    trigger_is_stuck_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "trigger_stuck",
      [this](std_msgs::msg::Empty::UniquePtr /*msg*/) {is_stuck_ = true;});

    reset_is_stuck_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "reset_stuck",
      [this](std_msgs::msg::Empty::UniquePtr /*msg*/) {is_stuck_ = false;});

    vel_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "tb3/cmd_vel",
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {current_vel_cmd_ = msg;});

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "tb3/odom",
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {current_velocity_ = msg;});
  }

  IsStuckCondition() = delete;

  ~IsStuckCondition()
  {
  }

  BT::NodeStatus tick() override
  {
    // Spin the node to get messages from the subscriptions
    rclcpp::spin_some(this->get_node_base_interface());

    if (isStuck()) {
      logMessage("tick(): Robot stuck!");
      // RCLCPP_WARN(get_logger(), "tick(): Robot stuck!");
      return BT::NodeStatus::SUCCESS;
    }

    logMessage("tick(): Robot not stuck");
    // RCLCPP_WARN(get_logger(), "tick(): Robot not stuck");
    return BT::NodeStatus::FAILURE;
  }

  bool isStuck()
  {
    using namespace std::chrono_literals;
    // TODO(orduno) Move the isStuck algorithm to the robot class. For that, make sure the other
    //              modules are using have the robot class, i.e. controller is not.
    // return robot_.isStuck();

    if (current_velocity_ == nullptr) {
      RCLCPP_WARN(get_logger(), "Initial odometry not yet received.");
      return false;
    }

    if (current_vel_cmd_ == nullptr) {
      RCLCPP_WARN(get_logger(), "No velocity command has been published.");
      return false;
    }

    // When the robot gets stuck it can have different types of motion (not moving at all,
    // random oscillations, etc). For now, we only address the case where the commanded
    // velocity is non-zero but the robot is not accelerating. A better approach would be to
    // forward simulate the robot motion according to the commanded velocity and compare it with
    // the actual motion.

    if (!is_stuck_) {
      // TODO(orduno) replace with actual odom error / vel fluctuation
      double odom_linear_vel_error = 0.0002;  // tuned using Gazebo + TB3
      double vel_cmd = current_vel_cmd_->linear.x;

      // TODO(orduno) assuming robot is moving forward
      if (vel_cmd > odom_linear_vel_error) {
        // Commanded velocity is non-zero

        // Assume the robot is free if it's accelerating forward
        double v1 = current_velocity_->twist.twist.linear.x;
        std::this_thread::sleep_for(1s);
        rclcpp::spin_some(this->get_node_base_interface());
        double v2 = current_velocity_->twist.twist.linear.x;

        if ((v2 + odom_linear_vel_error) < v1) {
          RCLCPP_WARN(get_logger(), "The robot is not accelerating, v1: %.6f, v2: %.6f", v1, v2);
          is_stuck_ = true;
        }
      }
    }

    return is_stuck_;
  }

  void logMessage(const std::string & msg) const
  {
    using namespace std::chrono_literals;

    // Log message once per second
    static auto time_since_msg = std::chrono::system_clock::now();
    auto current_time = std::chrono::system_clock::now();

    if (current_time - time_since_msg >= 1s) {
      RCLCPP_WARN(get_logger(), msg);
      time_since_msg = std::chrono::system_clock::now();
    }
  }

  void halt() override
  {
  }

private:
  bool is_stuck_;

  // Listen to odometry
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // The current velocity as received from the Odometry subscription
  std::shared_ptr<nav_msgs::msg::Odometry> current_velocity_;

  // Listen to the controller publishing velocity commands
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;
  // The last velocity command published by the controller
  std::shared_ptr<geometry_msgs::msg::Twist> current_vel_cmd_;

  // Trigger and reset for testing the stuck condition
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_is_stuck_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_is_stuck_sub_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__IS_STUCK_CONDITION_HPP
