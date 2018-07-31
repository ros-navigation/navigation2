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
#include "mission_execution/MissionExecution.hpp"

MissionExecution::MissionExecution(const std::string & name)
: MissionExecutionTaskServer(name)
{
  RCLCPP_INFO(get_logger(), "MissionExecution::MissionExecution");
  navigationTask_ = std::make_unique<NavigateToPoseTaskClient>("SimpleNavigator", this);
}

MissionExecution::~MissionExecution()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::~MissionExecution");
}

TaskStatus
MissionExecution::executeAsync(const MissionExecutionCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "MissionExecution:executeAsync");
  RCLCPP_INFO(get_logger(), "MissionExecution:executeAsync: plan: %s",
    command->mission_plan.c_str());

  // TODO(mjeronimo): Validate the mission plan for syntax and semantics

  // TODO(mjeronimo): Get the goal pose from the task in the mission plan
  auto goalPose = std::make_shared<NavigateToPoseCommand>();
  navigationTask_->executeAsync(goalPose);

  // Loop until navigation reaches a terminal state
  for (;; ) {
    // Check to see if this task (mission execution) has been canceled. If so,
    // cancel the navigation task first and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "MissionExecution::executeAsync: task has been canceled");
      navigationTask_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
    }

    // This task hasn't been canceled, so see if the navigation task has finished
    auto navResult = std::make_shared<NavigateToPoseResult>();
    TaskStatus status = navigationTask_->waitForResult(navResult, 100);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        {
          RCLCPP_INFO(get_logger(), "MissionExecution::executeAsync: navigation task completed");

          // No data to return from this task, just an empty result message
          MissionExecutionResult missionExecutionResult;
          setResult(missionExecutionResult);

          return TaskStatus::SUCCEEDED;
        }

      case TaskStatus::FAILED:
        return TaskStatus::FAILED;

      case TaskStatus::RUNNING:
        RCLCPP_INFO(get_logger(), "MissionExecution::executeAsync: navigation task still running");
        break;

      default:
        RCLCPP_INFO(get_logger(), "MissionExecution::executeAsync: invalid status value");
        throw std::logic_error("MissionExecution::executeAsync: invalid status value");
    }
  }
}
