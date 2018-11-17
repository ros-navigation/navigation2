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

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <ctime>
#include <iostream>

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

    static auto start_time = std::chrono::system_clock::now();
    auto end_time = std::chrono::system_clock::now();

    if ((end_time - start_time) % 1 >= 1.0) {
      std::cout << "IsStuckCondition::tick: Robot not stuck" << std::endl;
    }

    if (end_time - start_time > 5 /*seconds*/) {
      start_time = std::chrono::system_clock::now();
      std::cout << "IsStuckCondition::tick: Robot is stuck" << std::endl;
      return NodeStatus::SUCCEEDED;  // the condition was detected
    }

    return NodeStatus::FAILURE;  // the condition was not detected
  }

  void halt() override
  {
  }

};

private:

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__IS_STUCK_CONDITION_HPP
