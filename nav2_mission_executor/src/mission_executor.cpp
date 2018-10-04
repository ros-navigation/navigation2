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
#include "nav2_mission_executor/execute_mission_behavior_tree.hpp"

using nav2_tasks::TaskStatus;

namespace nav2_mission_executor
{

MissionExecutor::MissionExecutor()
: nav2_tasks::ExecuteMissionTaskServer("ExecuteMissionNode")
{
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("move_base_simple/goal",
      std::bind(&MissionExecutor::onGoalPoseReceived, this, std::placeholders::_1));

  plan_pub_ = create_publisher<nav2_msgs::msg::MissionPlan>(
    "ExecuteMissionTask_command");
}

void
MissionExecutor::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "MissionExecutor::onGoalPoseReceived");
  goal_pose_ = msg;

  auto message = nav2_msgs::msg::MissionPlan();
  message.mission_plan = "Hello, world!";

  RCLCPP_INFO(this->get_logger(), "MissionExecutor::onGoalPoseReceived: publishing a mission plan");
  plan_pub_->publish(message);
}

TaskStatus
MissionExecutor::execute(const nav2_tasks::ExecuteMissionCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "MissionExecutor:execute: plan: %s",
    command->mission_plan.c_str());

  // TODO(mjeronimo): Get the goal pose from the task in the mission plan. For now, we're
  // using the one received from rviz via the move_base_simple/goal topic.

  // Create and run the behavior tree for this mission
  ExecuteMissionBehaviorTree bt(shared_from_this());
  TaskStatus result = bt.run(std::bind(&MissionExecutor::cancelRequested, this));

  RCLCPP_INFO(get_logger(), "MissionExecutor::execute: completed: %d", result);
  return result;
}

}  // namespace nav2_mission_executor
