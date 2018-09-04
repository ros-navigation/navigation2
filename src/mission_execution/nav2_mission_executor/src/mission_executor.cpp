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

MissionExecutor::MissionExecutor(const std::string & name)
: nav2_tasks::ExecuteMissionTaskServer(name)
{
  RCLCPP_INFO(get_logger(), "MissionExecutor::MissionExecutor");
  navigationTask_ = std::make_unique<nav2_tasks::NavigateToPoseTaskClient>("SimpleNavigator", this);
}

MissionExecutor::~MissionExecutor()
{
  RCLCPP_INFO(get_logger(), "MissionExecutor::~MissionExecutor");
}

TaskStatus
MissionExecutor::execute(const nav2_tasks::ExecuteMissionCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "MissionExecutor:execute");
  RCLCPP_INFO(get_logger(), "MissionExecutor:execute: plan: %s",
    command->mission_plan.c_str());

  // TODO(mjeronimo): Validate the mission plan for syntax and semantics

  // TODO(mjeronimo): Get the goal pose from the task in the mission plan
  auto goalPose = std::make_shared<nav2_tasks::NavigateToPoseCommand>();
  navigationTask_->sendCommand(goalPose);

  auto navResult = std::make_shared<nav2_tasks::NavigateToPoseResult>();

  // Loop until navigation reaches a terminal state
  for (;; ) {
    // Check to see if this task (mission execution) has been canceled. If so,
    // cancel the navigation task first and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "MissionExecutor::execute: task has been canceled");
      navigationTask_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
    }

    // This task hasn't been canceled, so see if the navigation task has finished
    TaskStatus status = navigationTask_->waitForResult(navResult, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        {
          RCLCPP_INFO(get_logger(), "MissionExecutor::execute: navigation task completed");

          // No data to return from this task, just an empty result message
          nav2_tasks::ExecuteMissionResult result;
          setResult(result);

          return TaskStatus::SUCCEEDED;
        }

      case TaskStatus::FAILED:
        return TaskStatus::FAILED;

      case TaskStatus::RUNNING:
        RCLCPP_INFO(get_logger(), "MissionExecutor::execute: navigation task still running");
        break;

      default:
        RCLCPP_ERROR(get_logger(), "MissionExecutor::execute: invalid status value");
        throw std::logic_error("MissionExecutor::execute: invalid status value");
    }
  }
}

}  // namespace nav2_mission_execution
