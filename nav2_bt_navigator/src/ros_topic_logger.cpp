// Copyright (c) 2019 Intel Corporation
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

#include <memory>
#include <utility>
#include "nav2_bt_navigator/ros_topic_logger.hpp"
#include "tf2_ros/buffer_interface.h"

namespace nav2_bt_navigator
{

RosTopicLogger::RosTopicLogger(
  const rclcpp::Node::SharedPtr & ros_node, const BT::Tree & tree)
: StatusChangeLogger(tree.rootNode()), ros_node_(ros_node)
{
  log_pub_ = ros_node_->create_publisher<nav2_msgs::msg::BehaviorTreeLog>(
    "behavior_tree_log",
    rclcpp::QoS(10));
}

void RosTopicLogger::callback(
  BT::Duration timestamp,
  const BT::TreeNode & node,
  BT::NodeStatus prev_status,
  BT::NodeStatus status)
{
  nav2_msgs::msg::BehaviorTreeStatusChange event;

  // BT timestamps are a duration since the epoch. Need to convert to a time_point
  // before converting to a msg.
  event.timestamp = tf2_ros::toMsg(tf2::TimePoint(timestamp));
  event.node_name = node.name();
  event.previous_status = toStr(prev_status, false);
  event.current_status = toStr(status, false);
  event_log_.push_back(std::move(event));

  RCLCPP_DEBUG(
    ros_node_->get_logger(), "[%.3f]: %25s %s -> %s",
    std::chrono::duration<double>(timestamp).count(),
    node.name().c_str(),
    toStr(prev_status, true).c_str(),
    toStr(status, true).c_str() );
}

void RosTopicLogger::flush()
{
  if (event_log_.size() > 0) {
    auto log_msg = std::make_unique<nav2_msgs::msg::BehaviorTreeLog>();
    log_msg->timestamp = ros_node_->now();
    log_msg->event_log = event_log_;
    log_pub_->publish(std::move(log_msg));
    event_log_.clear();
  }
}

}   // namespace nav2_bt_navigator
