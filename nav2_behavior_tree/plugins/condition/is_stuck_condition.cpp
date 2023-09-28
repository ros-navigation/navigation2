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

#include <string>
#include <chrono>

#include "nav2_behavior_tree/plugins/condition/is_stuck_condition.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_behavior_tree
{

IsStuckCondition::IsStuckCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_stuck_(false),
  odom_history_size_(10),
  current_accel_(0.0),
  brake_accel_limit_(-10.0)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  callback_group_executor_thread = std::thread([this]() {callback_group_executor_.spin();});

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsStuckCondition::onOdomReceived, this, std::placeholders::_1),
    sub_option);

  RCLCPP_DEBUG(node_->get_logger(), "Initialized an IsStuckCondition BT node");

  RCLCPP_INFO_ONCE(node_->get_logger(), "Waiting on odometry");
}

IsStuckCondition::~IsStuckCondition()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsStuckCondition BT node");
  callback_group_executor_.cancel();
  callback_group_executor_thread.join();
}

void IsStuckCondition::onOdomReceived(const typename nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(node_->get_logger(), "Got odometry");

  while (odom_history_.size() >= odom_history_size_) {
    odom_history_.pop_front();
  }

  odom_history_.push_back(*msg);

  // TODO(orduno) #383 Move the state calculation and is stuck to robot class
  updateStates();
}

BT::NodeStatus IsStuckCondition::tick()
{
  // TODO(orduno) #383 Once check for is stuck and state calculations are moved to robot class
  //              this becomes
  // if (robot_state_.isStuck()) {

  if (is_stuck_) {
    logStuck("Robot got stuck!");
    return BT::NodeStatus::SUCCESS;  // Successfully detected a stuck condition
  }

  logStuck("Robot is free");
  return BT::NodeStatus::FAILURE;  // Failed to detected a stuck condition
}

void IsStuckCondition::logStuck(const std::string & msg) const
{
  static std::string prev_msg;

  if (msg == prev_msg) {
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
  prev_msg = msg;
}

void IsStuckCondition::updateStates()
{
  // Approximate acceleration
  // TODO(orduno) #400 Smooth out velocity history for better accel approx.
  if (odom_history_.size() > 2) {
    auto curr_odom = odom_history_.end()[-1];
    double curr_time = static_cast<double>(curr_odom.header.stamp.sec);
    curr_time += (static_cast<double>(curr_odom.header.stamp.nanosec)) * 1e-9;

    auto prev_odom = odom_history_.end()[-2];
    double prev_time = static_cast<double>(prev_odom.header.stamp.sec);
    prev_time += (static_cast<double>(prev_odom.header.stamp.nanosec)) * 1e-9;

    double dt = curr_time - prev_time;
    double vel_diff = static_cast<double>(
      curr_odom.twist.twist.linear.x - prev_odom.twist.twist.linear.x);
    current_accel_ = vel_diff / dt;
  }

  is_stuck_ = isStuck();
}

bool IsStuckCondition::isStuck()
{
  // TODO(orduno) #400 The robot getting stuck can result on different types of motion
  // depending on the state prior to getting stuck (sudden change in accel, not moving at all,
  // random oscillations, etc). For now, we only address the case where there is a sudden
  // harsh deceleration. A better approach to capture all situations would be to do a forward
  // simulation of the robot motion and compare it with the actual one.

  // Detect if robot bumped into something by checking for abnormal deceleration
  if (current_accel_ < brake_accel_limit_) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Current deceleration is beyond brake limit."
      " brake limit: %.2f, current accel: %.2f", brake_accel_limit_, current_accel_);

    return true;
  }

  return false;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsStuckCondition>("IsStuck");
}
