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

namespace nav2_behavior_tree
{

IsStoppedCondition::IsStoppedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  velocity_threshold_(0.01),
  duration_stopped_(1000ms),
  stopped_stamp_(rclcpp::Time(0, 0, RCL_ROS_TIME))
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  odom_smoother_ = config().blackboard->get<std::shared_ptr<nav2_util::OdomSmoother>>(
    "odom_smoother");
}

IsStoppedCondition::~IsStoppedCondition()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsStoppedCondition BT node");
}

BT::NodeStatus IsStoppedCondition::tick()
{
  getInput("velocity_threshold", velocity_threshold_);
  getInput("duration_stopped", duration_stopped_);

  auto twist = odom_smoother_->getRawTwistStamped();

  // if there is no timestamp, set it to now
  if (twist.header.stamp.sec == 0 && twist.header.stamp.nanosec == 0) {
    twist.header.stamp = node_->get_clock()->now();
  }

  if (abs(twist.twist.linear.x) < velocity_threshold_ &&
    abs(twist.twist.linear.y) < velocity_threshold_ &&
    abs(twist.twist.angular.z) < velocity_threshold_)
  {
    if (stopped_stamp_ == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
      stopped_stamp_ = rclcpp::Time(twist.header.stamp);
    }

    if (node_->get_clock()->now() - stopped_stamp_ > rclcpp::Duration(duration_stopped_)) {
      stopped_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::RUNNING;
    }

  } else {
    stopped_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsStoppedCondition>("IsStopped");
}
