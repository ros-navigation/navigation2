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

#ifndef NAV2_BEHAVIOR_TREE__TIME_EXPIRED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__TIME_EXPIRED_CONDITION_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"

#include "time_expired_condition.hpp"

namespace nav2_behavior_tree
{

TimeExpiredCondition::TimeExpiredCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  period_(1.0)
{
  getInput("seconds", period_);
}

BT::NodeStatus TimeExpiredCondition::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    start_ = std::chrono::high_resolution_clock::now();
    return BT::NodeStatus::FAILURE;
  }

  // Determine how long its been since we've started this iteration
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = now - start_;

  // Now, get that in seconds
  typedef std::chrono::duration<float> float_seconds;
  auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

  if (seconds.count() < period_) {
    return BT::NodeStatus::FAILURE;
  }

  start_ = std::chrono::high_resolution_clock::now();  // Reset the timer
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TimeExpiredCondition>("TimeExpired");
}

#endif  // NAV2_BEHAVIOR_TREE__TIME_EXPIRED_CONDITION_HPP_
