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
#include <vector>
#include "nav2_behavior_tree/plugins/condition/goal_updated_condition.hpp"

namespace nav2_behavior_tree
{

GoalUpdatedCondition::GoalUpdatedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_global_(false),
  current_run_id_("")
{
}

void GoalUpdatedCondition::initialize()
{
  auto node = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  is_global_ = node->declare_or_get_parameter("is_global", false);
}

BT::NodeStatus GoalUpdatedCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

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
      // Update snapshot so we can compare against the new run's goal, then fall through
      BT::getInputOrBlackboard("goals", goals_);
      BT::getInputOrBlackboard("goal", goal_);
    }
  } else {
    if (!BT::isStatusActive(status())) {
      BT::getInputOrBlackboard("goals", goals_);
      BT::getInputOrBlackboard("goal", goal_);
      return BT::NodeStatus::FAILURE;
    }
  }

  nav_msgs::msg::Goals current_goals;
  geometry_msgs::msg::PoseStamped current_goal;
  BT::getInputOrBlackboard("goals", current_goals);
  BT::getInputOrBlackboard("goal", current_goal);

  if (goal_ != current_goal || goals_ != current_goals) {
    goal_ = current_goal;
    goals_ = current_goals;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalUpdatedCondition>("GoalUpdated");
}
