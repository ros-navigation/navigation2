// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BT_NAVIGATOR__REACHED_GOAL_CONDITION_NODE_HPP_
#define NAV2_BT_NAVIGATOR__REACHED_GOAL_CONDITION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "behavior_tree_core/behavior_tree.h"
#include "nav2_tasks/follow_path_task.hpp"

namespace nav2_bt_navigator
{

class ReachedGoalConditionNode : public BT::ConditionNode
{
public:
  explicit ReachedGoalConditionNode(rclcpp::Node::SharedPtr node);
  ReachedGoalConditionNode() = delete;

  BT::NodeStatus tick() override;

private:
  // The task that we're monitoring for success
  nav2_tasks::FollowPathTaskClient taskClient_;

  // The result message returned by the task
  nav2_tasks::FollowPathResult::SharedPtr result_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__REACHED_GOAL_CONDITION_NODE_HPP_
