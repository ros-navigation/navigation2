// Copyright (c) 2020 Aitor Miguel Blanco
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

#include "nav2_behavior_tree/plugins/condition/goal_updated_condition.hpp"

namespace nav2_behavior_tree
{

GoalUpdatedCondition::GoalUpdatedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
}

BT::NodeStatus GoalUpdatedCondition::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    goal_ = config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal");
    return BT::NodeStatus::FAILURE;
  }

  auto current_goal = config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal");
  if (goal_ != current_goal) {
    goal_ = current_goal;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalUpdatedCondition>("GoalUpdated");
}
