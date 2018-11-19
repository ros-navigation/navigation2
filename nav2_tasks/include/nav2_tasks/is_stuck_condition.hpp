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
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "behavior_tree_core/condition_node.h"

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
  }

  IsStuckCondition() = delete;

  ~IsStuckCondition()
  {
  }

  BT::NodeStatus tick() override
  {
    using namespace std::chrono_literals;

    // TODO(orduno) Detect if robot is stuck
    //              i.e. compare the actual robot motion with the velocity command

    // Fake some time used for checking condition
    std::this_thread::sleep_for(50ms);

    // Spin the node to get messages from the subscriptions
    rclcpp::spin_some(this->get_node_base_interface());

    if (is_stuck_) {
      // if (current_time - time_since_msg >= 1s) {
        // RCLCPP_WARN(get_logger(), "tick(): Robot stuck!");
        // time_since_msg = std::chrono::system_clock::now();
      // }
      logMessage("tick(): Robot stuck!");
      return BT::NodeStatus::SUCCESS;
    }

    logMessage("tick(): Robot not stuck");
    // if (current_time - time_since_msg >= 1s) {
    //   RCLCPP_INFO(get_logger(), "tick(): Robot not stuck");
    //   time_since_msg = std::chrono::system_clock::now();
    // }

    return BT::NodeStatus::FAILURE;
  }

  void logMessage(const std::string & msg) const {
    using namespace std::chrono_literals;

    // Log messages once per second
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
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_is_stuck_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_is_stuck_sub_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__IS_STUCK_CONDITION_HPP
