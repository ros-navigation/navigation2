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

#include <string>
#include <memory>
#include <exception>
#include "nav2_mission_executor/mission_executor.hpp"
#include "nav2_mission_executor/execute_mission_behavior_tree.hpp"

using nav2_tasks::TaskStatus;

namespace nav2_mission_executor
{

MissionExecutor::MissionExecutor()
: nav2_tasks::ExecuteMissionTaskServer("ExecuteMissionNode")
{
}

MissionExecutor::~MissionExecutor()
{
}

TaskStatus
MissionExecutor::execute(const nav2_tasks::ExecuteMissionCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "MissionExecutor:execute: plan: %s",
    command->mission_plan.c_str());

  // TODO(mjeronimo): Validate the mission plan for syntax and semantics

  // Create the behavior tree for this mission
  ExecuteMissionBehaviorTree bt(this);

  // Compose the NavigateToPose message for the Navigation module
  auto navigateToPoseCommand = std::make_shared<nav2_tasks::NavigateToPoseCommand>();

  // TODO(mjeronimo): Get the goal pose from the task in the mission plan. For now,
  // fake out some values
  navigateToPoseCommand->pose.position.x = 0;
  navigateToPoseCommand->pose.position.y = 1;
  navigateToPoseCommand->pose.position.z = 2;
  navigateToPoseCommand->pose.orientation.x = 0;
  navigateToPoseCommand->pose.orientation.y = 1;
  navigateToPoseCommand->pose.orientation.z = 2;
  navigateToPoseCommand->pose.orientation.w = 3;

  TaskStatus result = bt.run(navigateToPoseCommand);
  RCLCPP_INFO(get_logger(), "MissionExecutor::executeAsync: completed: %d", result);

  return result;
}

}  // namespace nav2_mission_executor
