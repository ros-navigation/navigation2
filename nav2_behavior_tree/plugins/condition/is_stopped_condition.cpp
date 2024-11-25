// Copyright (c) 2024 Angsa Robotics
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

#include "nav2_behavior_tree/plugins/condition/is_stopped_condition.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_behavior_tree
{

IsStoppedCondition::IsStoppedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_stopped_(false),
  velocity_threshold_(0.1),
  time_stopped_threshold_(1000),
  stopped_stamp_(rclcpp::Time(0))
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  getInput("topic_name", topic_name_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  callback_group_executor_thread = std::thread([this]() {callback_group_executor_.spin();});

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    topic_name_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsStoppedCondition::onOdomReceived, this, std::placeholders::_1),
    sub_option);
}

IsStoppedCondition::~IsStoppedCondition()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsStoppedCondition BT node");
  callback_group_executor_.cancel();
  callback_group_executor_thread.join();
}

void IsStoppedCondition::onOdomReceived(const typename nav_msgs::msg::Odometry::SharedPtr msg)
{
  getInput("velocity_threshold", velocity_threshold_);
  getInput("time_stopped_threshold", time_stopped_threshold_);

  // Check if the robot is stopped for a certain amount of time
  if (msg->twist.twist.linear.x < velocity_threshold_ &&
    msg->twist.twist.linear.y < velocity_threshold_ &&
    msg->twist.twist.angular.z < velocity_threshold_)
  {
    if (stopped_stamp_ == rclcpp::Time(0)) {
      stopped_stamp_ = rclcpp::Time(msg->header.stamp);
    } else if (rclcpp::Time(msg->header.stamp) - stopped_stamp_ >
      rclcpp::Duration(time_stopped_threshold_))
    {
      is_stopped_ = true;
    }
  } else {
    stopped_stamp_ = rclcpp::Time(0);

    std::lock_guard<std::mutex> lock(mutex_);
    is_stopped_ = false;
  }
}

BT::NodeStatus IsStoppedCondition::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_stopped_) {
    return BT::NodeStatus::SUCCESS;
  } else if (stopped_stamp_ != rclcpp::Time(0)) {
    // Robot was stopped but not for long enough
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsStoppedCondition>("IsStopped");
}
