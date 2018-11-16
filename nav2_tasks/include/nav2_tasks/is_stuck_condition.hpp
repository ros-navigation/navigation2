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

#ifndef NAV2_TASKS__IS_STUCK_CONDITION_HPP
#define NAV2_TASKS__IS_STUCK_CONDITION_HPP

#include <string>

#include "behavior_tree_core/condition_node.h"

namespace nav2_tasks
{

class IsStuckCondition : public BT::ConditionNode
{
public:
  explicit IsStuckCondition(const std::string & condition_name)
  : BT::ConditionNode(condition_name)
  {
  }

  IsStuckCondition() = delete;

  ~IsStuckCondition()
  {
  }

  BT::NodeStatus tick() override
  {
    // TODO(orduno) Write code for detecting if the robot is stuck
    //              i.e. compare the actual robot motion with the velocity command

    // For testing, let's return true (or success on BT terminology)
    std::cout << "IsStuckCondition::tick: the Condition is true" << std::endl;

    return NodeStatus::SUCCESS;
  }

  void halt() override
  {
  }

};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__IS_STUCK_CONDITION_HPP
