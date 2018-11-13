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

#include "nav2_mission_executor/mission_executor.hpp"

#include <memory>
#include "nav2_mission_executor/execute_mission_behavior_tree.hpp"

using nav2_tasks::TaskStatus;

namespace nav2_mission_executor
{

MissionExecutor::MissionExecutor()
: Node("MissionExecutor")
{
  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](auto) {});

  task_server_ = std::make_unique<nav2_tasks::ExecuteMissionTaskServer>(temp_node);

  task_server_->setExecuteCallback(
    std::bind(&MissionExecutor::executeMission, this, std::placeholders::_1));
}

TaskStatus
MissionExecutor::executeMission(const nav2_tasks::ExecuteMissionCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "Executing mission plan: %s", command->mission_plan.c_str());

  // Create the blackboard that will be shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create<BT::BlackboardLocal>();

  // Create and run the behavior tree for this mission
  ExecuteMissionBehaviorTree bt(shared_from_this());
  TaskStatus result = bt.run(blackboard, command->mission_plan,
      std::bind(&nav2_tasks::ExecuteMissionTaskServer::cancelRequested,
      task_server_.get()));

  RCLCPP_INFO(get_logger(), "Completed mission execution: %d", result);
  return result;
}

}  // namespace nav2_mission_executor
