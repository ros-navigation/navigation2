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
    is_stuck_(false),
    test_mode_(true)
  {
    RCLCPP_INFO(get_logger(), "IsStuckCondition::constructor");

    trigger_is_stuck_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "trigger_stuck",
      [this](std_msgs::msg::Empty::UniquePtr /*msg*/) {is_stuck_ = true;});

    reset_is_stuck_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "reset_stuck",
      [this](std_msgs::msg::Empty::UniquePtr /*msg*/) {is_stuck_ = false;});

    vel_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {current_vel_cmd_ = msg;});

    auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    robot_ = std::make_unique<nav2_robot::Robot>(temp_node);
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
      return BT::NodeStatus::SUCCESS;
    }

    logMessage("tick(): Robot not stuck");
    return BT::NodeStatus::FAILURE;
  }

  bool isStuck()
  {
    using namespace std::chrono_literals;

    if (test_mode_) {
      // Fake some time used for checking condition
      std::this_thread::sleep_for(50ms);
      return is_stuck_;
    }

    auto current_velocity = std::make_shared<nav_msgs::msg::Odometry>();
    robot_->getCurrentVelocity(current_velocity);

    // TODO(orduno) For now we only address the case where the commanded velocity is non-zero
    //              but the robot is close to not moving.

    double threshold = 0.01;
    if (current_vel_cmd_->linear.x != 0.0) {
      if (current_velocity->twist.linear.x < threshold) {
        return true;
      }
    }

    return false;
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
  bool test_mode_;

  std::unique_ptr<nav2_robot::Robot> robot_;

  // The last velocity command published by the controller
  std::shared_ptr<geometry_msgs::msg::Twist> current_vel_cmd_;

  // Listen to controller publishing velocity commands
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;

  // Trigger and reset for testing the condition
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_is_stuck_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_is_stuck_sub_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__IS_STUCK_CONDITION_HPP
