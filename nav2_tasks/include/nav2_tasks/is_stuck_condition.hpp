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
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "behaviortree_cpp/condition_node.h"
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
    new_cmd_(false),
    odom_history_size_(10),
    cmd_history_size_(10),
    current_accel_(0.0),
    minimum_measured_accel_(0.0),
    brake_accel_limit_(-10.0)
  {
    RCLCPP_INFO(get_logger(), "IsStuckCondition::constructor");

    // Capture velocity commands published by other nodes
    // TODO(orduno) #381 Currently DWB is publishing commands directly, not using the Robot class.
    //              #383 Once all nodes use the Robot class we can change this as well.

    vel_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",
        [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(msg_mutex_);

          while (cmd_history_.size() >= cmd_history_size_) {
            cmd_history_.pop_front();
          }

          cmd_history_.push_back(*msg);
          new_cmd_ = true;
        }
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(msg_mutex_);

          while (odom_history_.size() >= odom_history_size_) {
            odom_history_.pop_front();
          }

          odom_history_.push_back(*msg);
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
      logStuck("Robot got stuck!");
      update_stuck_ = true;
      return BT::NodeStatus::SUCCESS;  // Successfully detected a stuck condition
    }

    logStuck("Robot is free");
    return BT::NodeStatus::FAILURE;  // Failed to detected a stuck condition
  }

  void logStuck(const std::string & msg) const
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
    while (spinning_ok_) {
      // Spin the node to get messages from the subscriptions
      rclcpp::spin_some(this->get_node_base_interface());

      while (!new_odom_) {
        RCLCPP_INFO_ONCE(get_logger(), "Waiting on odometry");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        rclcpp::spin_some(this->get_node_base_interface());
      }

      RCLCPP_INFO_ONCE(get_logger(), "Got odometry");

      updateStates();

      // Check if the robot got stuck and change state, only if it was already reported
      if (update_stuck_) {
        is_stuck_ = isStuck();

        // TODO(orduno) #383 Move algorithm to the robot class, perhaps with the rest of the thread
        // is_stuck_ = robot_.isStuck();

        if (is_stuck_) {
          update_stuck_ = false;
        }
      }
    }
  }

  bool isStuck()
  {
    std::unique_lock<std::mutex> lock(msg_mutex_);

    // TODO(orduno) #400 The robot getting stuck can result on different types of motion
    // depending on the state prior to getting stuck (sudden change in accel, not moving at all,
    // random oscillations, etc). For now, we only address the case where there is a sudden
    // harsh deceleration. A better approach to capture all situations would be to do a forward
    // simulation of the robot motion and compare it with the actual one.

    // Detect if robot bumped into something by checking for abnormal deceleration
    if (current_accel_ < brake_accel_limit_) {
      RCLCPP_INFO_ONCE(get_logger(), "Current acceleration is below brake limit."
        " brake limit: %.2f, current accel: %.2f", brake_accel_limit_, current_accel_);
    }

    return false;
  }

  void updateStates()
  {
    // Approximate acceleration
    // TODO(orduno) #400 Smooth out velocity history for better accel approx.
    if (odom_history_.size() > 2) {
      auto curr_odom = odom_history_.end()[-1];
      double t2 = static_cast<double>(curr_odom.header.stamp.sec);
      t2 += (static_cast<double>(curr_odom.header.stamp.nanosec)) * 1e-9;

      auto prev_odom = odom_history_.end()[-2];
      double t1 = static_cast<double>(prev_odom.header.stamp.sec);
      t1 += (static_cast<double>(prev_odom.header.stamp.nanosec)) * 1e-9;

      double dt = t2 - t1;
      double vel_diff = static_cast<double>(
        curr_odom.twist.twist.linear.x - prev_odom.twist.twist.linear.x);
      current_accel_ = vel_diff / dt;

      if (current_accel_ < minimum_measured_accel_) {
        minimum_measured_accel_ = current_accel_;
        RCLCPP_DEBUG(get_logger(),
          "Minimum accel detected, dt: %.6f s, vel diff: %.6f m/s, accel: %.6f m/s^2",
          dt, vel_diff, current_accel_);
      }
    }
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
  // Store history of odometry measurements
  std::deque<nav_msgs::msg::Odometry> odom_history_;
  std::deque<nav_msgs::msg::Odometry>::size_type odom_history_size_;

  // Listen to the controller publishing velocity commands
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;
  // Store history of velocity commands
  std::deque<geometry_msgs::msg::Twist> cmd_history_;
  std::deque<geometry_msgs::msg::Twist>::size_type cmd_history_size_;

  // Calculated states
  double current_accel_;
  double minimum_measured_accel_;

  // Robot specific paramters
  double brake_accel_limit_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__IS_STUCK_CONDITION_HPP_
