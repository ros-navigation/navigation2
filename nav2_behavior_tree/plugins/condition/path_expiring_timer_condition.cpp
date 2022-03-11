// Copyright (c) 2022 Joshua Wallace
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
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"

#include "nav2_behavior_tree/plugins/condition/path_expiring_timer_condition.hpp"

namespace nav2_behavior_tree
{

PathExpiringTimerCondition::PathExpiringTimerCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  period_(1.0),
  first_time_(true)
{
  getInput("seconds", period_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus PathExpiringTimerCondition::tick()
{
  if (first_time_) {
    getInput("path", prev_path_);
    first_time_ = false;
    start_ = node_->now();
    return BT::NodeStatus::FAILURE;
  }

  // Grab the new path
  nav_msgs::msg::Path path;
  getInput("path", path);

  // Reset timer if the path has been updated
  if (prev_path_ != path) {
    prev_path_ = path;
    start_ = node_->now();
  }

  // Determine how long its been since we've started this iteration
  auto elapsed = node_->now() - start_;

  // Now, get that in seconds
  auto seconds = elapsed.seconds();

  if (seconds < period_) {
    return BT::NodeStatus::FAILURE;
  }

  start_ = node_->now();  // Reset the timer
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathExpiringTimerCondition>("PathExpiringTimer");
}
