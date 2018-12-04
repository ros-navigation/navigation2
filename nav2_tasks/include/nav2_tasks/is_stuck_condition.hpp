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
#include <atomic>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "behavior_tree_core/condition_node.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_tasks
{

class IsStuckCondition : public BT::ConditionNode, public rclcpp::Node
{
public:
  explicit IsStuckCondition(const std::string & condition_name)
  : BT::ConditionNode(condition_name),
    Node("IsStuckCondition"),
    workerThread_(nullptr),
    is_stuck_(false),
    update_stuck_(true),
    spinning_ok_(false),
    new_odom_(false),
    new_cmd_(false)
  {
    RCLCPP_INFO(get_logger(), "IsStuckCondition::constructor");

    // Capture velocity commands published by other nodes
    // TODO(orduno) #381 Currently DWB is publishing commands directly, not using the Robot class.
    //              #383 Once all nodes use the Robot class we can change this as well.

    vel_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",
        [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(msg_mutex_);

          if (current_vel_cmd_ != nullptr) {
            // Keep track of the previous *different* forward velocity command
            if (previous_vel_cmd_->linear.x != current_vel_cmd_->linear.x) {
              previous_vel_cmd_ = current_vel_cmd_;
            }
          } else {
            // We set current and previous to same value at startup
            previous_vel_cmd_ = msg;
          }

          current_vel_cmd_ = msg;
          new_cmd_ = true;
      }
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(msg_mutex_);
        current_velocity_ = msg;
        new_odom_ = true;
      }
    );

    startWorkerThread();
  }

  IsStuckCondition() = delete;

  ~IsStuckCondition()
  {
    stopWorkerThread();
  }

  BT::NodeStatus tick() override
  {
    if (is_stuck_) {
      logMessage("tick(): Robot stuck!");
      update_stuck_ = true;
      return BT::NodeStatus::SUCCESS;  // Successfully detected a stuck condition
    }

    logMessage("tick(): Robot not stuck");
    return BT::NodeStatus::FAILURE;  // Failed to detected a stuck condition
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

  void startWorkerThread()
  {
    spinning_ok_ = true;
    workerThread_ = new std::thread(&IsStuckCondition::workerThread, this);
  }

  void stopWorkerThread()
  {
    spinning_ok_ = false;
    workerThread_->join();
    delete workerThread_;
    workerThread_ = nullptr;
  }

  void workerThread()
  {
    while(spinning_ok_)
    {
      // Spin the node to get messages from the subscriptions
      rclcpp::spin_some(this->get_node_base_interface());

      // Check if the robot got stuck and change state, only if it was already reported
      if (update_stuck_) {
        is_stuck_ = isStuck();

        // TODO(orduno) #383 Move algorithm to the robot class
        // is_stuck_ = robot_.isStuck();

        update_stuck_ = false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  bool isStuck()
  {
    std::unique_lock<std::mutex> lock(msg_mutex_);

    if (current_velocity_ == nullptr) {
      RCLCPP_WARN_ONCE(get_logger(), "Can't check if stuck, "
      "initial odometry not yet received.");
      return false;
    }

    if (current_vel_cmd_ == nullptr) {
      RCLCPP_WARN_ONCE(get_logger(), "Can't check if stuck, "
        "velocity commands have not been published.");
      return false;
    }

    RCLCPP_INFO_ONCE(get_logger(), "Initial odometry and velocity commands received.");

    // TODO(orduno) #400 The robot getting stuck can result on different types of motion
    // depending on the state prior to getting stuck (sudden change in accel, not moving at all,
    // random oscillations, etc). For now, we only address the case where the commanded velocity
    // is non-zero but the robot is not accelerating. A better approach is to do a forward
    // simulation of the robot motion and compare it with the actual one.

    #if 0

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
        // Robot should be decelerating
        if ((v2 - tolerance - odom_linear_vel_error) > v1) {
          RCLCPP_WARN(get_logger(),
            "The robot is not decelerating, previous cmd: %.6f, current cmd: %.6f,"
            "  v1: %.6f, v2: %.6f", prev_cmd, curr_cmd, v1, v2);
          return true;
        }
      }
    }
    #endif

    return false;
  }

  void halt() override
  {
  }

private:
  // We handle the detection of the stuck condition on a separate thread
  std::thread * workerThread_;

  std::atomic<bool> is_stuck_;
  std::atomic<bool> update_stuck_;
  std::atomic<bool> spinning_ok_;

  std::mutex msg_mutex_;
  std::atomic<bool> new_odom_;
  std::atomic<bool> new_cmd_;

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
