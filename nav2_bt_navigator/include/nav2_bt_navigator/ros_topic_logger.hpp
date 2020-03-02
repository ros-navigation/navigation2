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

#ifndef NAV2_BT_NAVIGATOR__ROS_TOPIC_LOGGER_HPP_
#define NAV2_BT_NAVIGATOR__ROS_TOPIC_LOGGER_HPP_

#include <vector>
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav2_msgs/msg/behavior_tree_status_change.h"

namespace nav2_bt_navigator
{

class RosTopicLogger : public BT::StatusChangeLogger
{
public:
  RosTopicLogger(const rclcpp::Node::SharedPtr & ros_node, const BT::Tree & tree);

  void callback(
    BT::Duration timestamp,
    const BT::TreeNode & node,
    BT::NodeStatus prev_status,
    BT::NodeStatus status) override;

  void flush() override;

protected:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr log_pub_;
  std::vector<nav2_msgs::msg::BehaviorTreeStatusChange> event_log_;
};

}   // namespace nav2_bt_navigator

#endif   // NAV2_BT_NAVIGATOR__ROS_TOPIC_LOGGER_HPP_
