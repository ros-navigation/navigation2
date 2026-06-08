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

#include "behaviortree_cpp/condition_node.h"

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
  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
}

BT::NodeStatus PathExpiringTimerCondition::tick()
{
  if (first_time_) {
    getInput("seconds", period_);
    getInput("path", prev_path_ptr_);
    first_time_ = false;
    start_ = node_->now();
    return BT::NodeStatus::FAILURE;
  }

  // Grab the new path
  std::shared_ptr<nav_msgs::msg::Path> path_ptr;
  getInput("path", path_ptr);

  // Reset timer if the path has been updated.
  // Pointer equality: same shared_ptr = same path object — skip O(N) value comparison
  if (path_ptr && prev_path_ptr_ &&
    path_ptr.get() != prev_path_ptr_.get() &&
    *path_ptr != *prev_path_ptr_)
  {
    prev_path_ptr_ = path_ptr;
    start_ = node_->now();
  } else if (path_ptr && !prev_path_ptr_) {
    prev_path_ptr_ = path_ptr;
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

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathExpiringTimerCondition>("PathExpiringTimer");
}
