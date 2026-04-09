// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#include "nav2_behavior_tree/plugins/condition/time_expired_condition.hpp"

namespace nav2_behavior_tree
{

TimeExpiredCondition::TimeExpiredCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  period_(1.0),
  is_global_(false),
  current_run_id_("")
{
}

void TimeExpiredCondition::initialize()
{
  getInput("seconds", period_);
  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  is_global_ = node_->declare_or_get_parameter("is_global", false);
}

BT::NodeStatus TimeExpiredCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
    if (!is_global_) {
      // Local mode: reset timer and return FAILURE on first tick after halt
      start_ = node_->now();
      return BT::NodeStatus::FAILURE;
    } else if (!start_.nanoseconds()) {
      // Global mode: initialize start_ only on the very first tick ever
      start_ = node_->now();
    }
  }

  // Global mode: on RunID change fall through without resetting start_
  if (is_global_) {
    std::string new_run_id;
    try {
      new_run_id = config().blackboard->template get<std::string>("run_id");
    } catch (const std::exception & e) {
      throw std::runtime_error(
        "is_global=true requires 'run_id' to be set on the blackboard for condition: " +
        std::string(name()));
    }
    if (new_run_id != current_run_id_) {
      current_run_id_ = new_run_id;
      // Do not reset start_ to preserve time measurement across runs
    }
  }

  // Determine how long it's been since we've started this iteration
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
  factory.registerNodeType<nav2_behavior_tree::TimeExpiredCondition>("TimeExpired");
}
