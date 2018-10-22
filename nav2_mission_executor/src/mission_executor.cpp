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

using namespace std::chrono_literals;
using nav2_tasks::TaskStatus;

namespace nav2_mission_execution
{

MissionExecutor::MissionExecutor()
: nav2_tasks::ExecuteMissionTaskServer("ExecuteMissionNode")
{
  RCLCPP_INFO(get_logger(), "Initializing MissionExecutor.");
  navTaskClient_ = std::make_unique<nav2_tasks::NavigateToPoseTaskClient>(this);

  if (!navTaskClient_->waitForServer(nav2_tasks::defaultServerTimeout)) {
    RCLCPP_ERROR(get_logger(), "MissionExecutor: NavigateToPoseTaskServer not running!");
    throw std::runtime_error("MissionExecutor: NavigateToPoseTaskServer not running");
  }

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("move_base_simple/goal",
      std::bind(&MissionExecutor::onGoalPoseReceived, this, std::placeholders::_1));

  plan_pub_ = create_publisher<nav2_msgs::msg::MissionPlan>(
    "ExecuteMissionTask_command");
}

MissionExecutor::~MissionExecutor()
{
  RCLCPP_INFO(get_logger(), "Shutting down MissionExecutor");
}

void
MissionExecutor::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_pose_ = msg;

  auto message = nav2_msgs::msg::MissionPlan();
  message.mission_plan = "Hello, world!";

  RCLCPP_INFO(this->get_logger(), "MissionExecutor: Publishing a new mission plan");
  plan_pub_->publish(message);
}

TaskStatus
MissionExecutor::execute(const nav2_tasks::ExecuteMissionCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "MissionExecutor: Executing task: %s from %f",
    command->mission_plan.c_str(), command->header.stamp);

  // TODO(mjeronimo): Validate the mission plan for syntax and semantics

  // TODO(mjeronimo): Get the goal pose from the task in the mission plan. For now, we're
  // using the one received from rviz via the move_base_simple/goal topic.
  navTaskClient_->sendCommand(goal_pose_);

  auto navResult = std::make_shared<nav2_tasks::NavigateToPoseResult>();

  // Loop until navigation reaches a terminal state
  for (;; ) {
    // Check to see if this task (mission execution) has been canceled. If so,
    // cancel the navigation task first and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "MissionExecutor: Task %s has been canceled.",
        command->mission_plan.c_str());
      navTaskClient_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
    }

    // This task hasn't been canceled, so see if the navigation task has finished
    TaskStatus status = navTaskClient_->waitForResult(navResult, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        {
          RCLCPP_INFO(get_logger(), "MissionExecutor: Mission task %s completed",
            command->mission_plan.c_str());

          // No data to return from this task, just an empty result message
          nav2_tasks::ExecuteMissionResult result;
          setResult(result);

          return TaskStatus::SUCCEEDED;
        }

      case TaskStatus::FAILED:
        RCLCPP_ERROR(get_logger(), "MissionExecutor::execute: navigation task failed");
        return TaskStatus::FAILED;

      case TaskStatus::RUNNING:
        RCLCPP_INFO(get_logger(), "MissionExecutor: Current mission task still running.");
        break;

      default:
        RCLCPP_ERROR(get_logger(), "MissionExecutor: Invalid status value.");
        throw std::logic_error("MissionExecutor::execute: invalid status value");
    }
  }
}

}  // namespace nav2_mission_execution
