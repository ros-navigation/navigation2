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

#ifndef NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_
#define NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_

#include <vector>
#include <memory>
#include <utility>
#include <string>

#include "behaviortree_cpp/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav2_msgs/msg/behavior_tree_status_change.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer_interface.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A class to publish BT logs on BT status change
 */
class RosTopicLogger : public BT::StatusChangeLogger
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RosTopicLogger
   * @param ros_node Weak pointer to parent nav2::LifecycleNode
   * @param tree BT to monitor
   * @param log_idle Whether to enable logging transitions to IDLE state
   */
  RosTopicLogger(
    const nav2::LifecycleNode::WeakPtr & ros_node,
    const BT::Tree & tree,
    bool log_idle = true)
  : StatusChangeLogger(tree.rootNode())
  {
    auto node = ros_node.lock();
    clock_ = node->get_clock();
    logger_ = node->get_logger().get_child("ros_topic_logger");
    log_pub_ = node->create_publisher<nav2_msgs::msg::BehaviorTreeLog>(
      "behavior_tree_log");
    enableTransitionToIdle(log_idle);
  }

  /**
   * @brief Callback function which is called each time BT changes status
   * @param timestamp Timestamp of BT status change
   * @param node Node that changed status
   * @param prev_status Previous status of the node
   * @param status Current status of the node
   */
  void callback(
    BT::Duration timestamp,
    const BT::TreeNode & node,
    BT::NodeStatus prev_status,
    BT::NodeStatus status) override
  {
    nav2_msgs::msg::BehaviorTreeStatusChange event;

    // BT timestamps are a duration since the epoch. Need to convert to a time_point
    // before converting to a msg.
    event.timestamp = tf2_ros::toMsg(tf2::TimePoint(timestamp));
    event.node_name = node.name();
    event.uid = node.UID();
    event.previous_status = toStr(prev_status, false);
    event.current_status = toStr(status, false);
    event_log_.push_back(std::move(event));

    auto prev_pad = std::string(kStatusWidth - toStr(prev_status, false).size(), ' ');
    auto curr_pad = std::string(kStatusWidth - toStr(status, false).size(), ' ');
    RCLCPP_DEBUG(
      logger_, "[%.3f]: %s%s -> %s%s  %s",
      std::chrono::duration<double>(timestamp).count(),
      toStr(prev_status, true).c_str(), prev_pad.c_str(),
      toStr(status, true).c_str(), curr_pad.c_str(),
      node.name().c_str() );
  }

  /**
   * @brief Clear log buffer if any
   */
  void flush() override
  {
    if (!event_log_.empty()) {
      auto log_msg = std::make_unique<nav2_msgs::msg::BehaviorTreeLog>();
      log_msg->timestamp = clock_->now();
      log_msg->event_log = event_log_;
      log_pub_->publish(std::move(log_msg));
      event_log_.clear();
    }
  }

protected:
  // Longest BT status string is 7 chars (RUNNING, SUCCESS, FAILURE); pad shorter ones (IDLE)
  static constexpr size_t kStatusWidth = 7;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("bt_navigator")};
  rclcpp::Publisher<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr log_pub_;
  std::vector<nav2_msgs::msg::BehaviorTreeStatusChange> event_log_;
};

}   // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_
