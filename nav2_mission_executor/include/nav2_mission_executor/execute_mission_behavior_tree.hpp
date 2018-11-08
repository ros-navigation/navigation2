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

#ifndef NAV2_MISSION_EXECUTOR__EXECUTE_MISSION_BEHAVIOR_TREE_HPP_
#define NAV2_MISSION_EXECUTOR__EXECUTE_MISSION_BEHAVIOR_TREE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_tasks/behavior_tree_engine.hpp"

namespace nav2_mission_executor
{

class ExecuteMissionBehaviorTree : public nav2_tasks::BehaviorTreeEngine
{
public:
  explicit ExecuteMissionBehaviorTree(rclcpp::Node::SharedPtr node);
  ExecuteMissionBehaviorTree() = delete;
};

}  // namespace nav2_mission_executor

#endif  // NAV2_MISSION_EXECUTOR__EXECUTE_MISSION_BEHAVIOR_TREE_HPP_
