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
#include <cmath>
#include <thread>
#include <atomic>
#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
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
    spinning_ok_(false),
    odom_history_size_(10),
    current_accel_(0.0),
    brake_accel_limit_(-10.0)
  {
    RCLCPP_DEBUG(get_logger(), "Creating an IsStuckCondition BT node");

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",
        std::bind(&IsStuckCondition::onOdomReceived, this, std::placeholders::_1));

    RCLCPP_INFO_ONCE(get_logger(), "Waiting on odometry");

    startWorkerThread();
  }

  IsStuckCondition() = delete;

  ~IsStuckCondition()
  {
    RCLCPP_DEBUG(this->get_logger(), "Shutting down IsStuckCondition BT node");
    stopWorkerThread();
  }

  void onOdomReceived(const typename nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO_ONCE(get_logger(), "Got odometry");

    while (odom_history_.size() >= odom_history_size_) {
      odom_history_.pop_front();
    }

    odom_history_.push_back(*msg);

    // TODO(orduno) #383 Move the state calculation and is stuck to robot class
    updateStates();
    is_stuck_ = isStuck();
  }

  BT::NodeStatus tick() override
  {
    // TODO(orduno) #383 Once check for is stuck and state calculations are moved to robot class
    //              this becomes
    // if (robot_.isStuck()) {

    if (is_stuck_) {
      logStuck("Robot got stuck!");
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
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  bool isStuck()
  {
    // TODO(orduno) #400 The robot getting stuck can result on different types of motion
    // depending on the state prior to getting stuck (sudden change in accel, not moving at all,
    // random oscillations, etc). For now, we only address the case where there is a sudden
    // harsh deceleration. A better approach to capture all situations would be to do a forward
    // simulation of the robot motion and compare it with the actual one.

    // Detect if robot bumped into something by checking for abnormal deceleration
    if (current_accel_ < brake_accel_limit_) {
      RCLCPP_DEBUG(get_logger(), "Current deceleration is beyond brake limit."
        " brake limit: %.2f, current accel: %.2f", brake_accel_limit_, current_accel_);

      return true;
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
    }
  }

  void halt() override
  {
  }

private:
  // We handle the detection of the stuck condition on a separate thread
  std::thread * workerThread_;
  std::atomic<bool> is_stuck_;
  std::atomic<bool> spinning_ok_;

  // Listen to odometry
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // Store history of odometry measurements
  std::deque<nav_msgs::msg::Odometry> odom_history_;
  std::deque<nav_msgs::msg::Odometry>::size_type odom_history_size_;

  // Calculated states
  double current_accel_;

  // Robot specific paramters
  double brake_accel_limit_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__IS_STUCK_CONDITION_HPP_
