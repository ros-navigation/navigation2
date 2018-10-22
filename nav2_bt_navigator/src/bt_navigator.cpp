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
#include <chrono>
#include "nav2_bt_navigator/bt_navigator.hpp"

using namespace std::chrono_literals;
using nav2_tasks::TaskStatus;

namespace nav2_bt_navigator
{

BtNavigator::BtNavigator()
: nav2_tasks::NavigateToPoseTaskServer("NavigateToPoseNode")
{
  RCLCPP_INFO(get_logger(), "Initializing BtNavigator");

  plannerTaskClient_ = std::make_unique<nav2_tasks::ComputePathToPoseTaskClient>(this);
  controllerTaskClient_ = std::make_unique<nav2_tasks::FollowPathTaskClient>(this);

  if (!plannerTaskClient_->waitForServer(nav2_tasks::defaultServerTimeout)) {
    RCLCPP_ERROR(get_logger(), "BtNavigator: Planner is not running");
    throw std::runtime_error("BtNavigator: planner not running");
  }

  if (!controllerTaskClient_->waitForServer(nav2_tasks::defaultServerTimeout)) {
    RCLCPP_ERROR(get_logger(), "BtNavigator: Controller is not running");
    throw std::runtime_error("BtNavigator: controller not running");
  }
}

BtNavigator::~BtNavigator()
{
  RCLCPP_INFO(get_logger(), "Shutting down BtNavigator");
}

TaskStatus
BtNavigator::execute(const nav2_tasks::NavigateToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "BtNavigator: Received new navigation goal to (%.2f, %.2f).",
    command->pose.position.x, command->pose.position.y);

  // Compose the PathEndPoints message for Navigation
  auto endpoints = std::make_shared<nav2_tasks::ComputePathToPoseCommand>();
  // TODO(mjeronimo): get the starting pose from Localization (fake it out for now)
  endpoints->start = command->pose;
  endpoints->goal = command->pose;

  RCLCPP_INFO(get_logger(), "BtNavigator: Getting a path from the planner.");
  auto path = std::make_shared<nav2_tasks::ComputePathToPoseResult>();
  plannerTaskClient_->sendCommand(endpoints);

  // Loop until the subtasks are completed
  for (;; ) {
    // Check to see if this task (navigation) has been canceled. If so, cancel any child
    // tasks and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "BtNavigator: Task has been canceled.");
      plannerTaskClient_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
    }

    // Check if the planning task has completed
    TaskStatus status = plannerTaskClient_->waitForResult(path, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "BtNavigator: Planning task completed.");
        goto here;

      case TaskStatus::FAILED:
        return TaskStatus::FAILED;

      case TaskStatus::RUNNING:
        RCLCPP_DEBUG(get_logger(), "BtNavigator: Planning task still running");
        break;

      default:
        RCLCPP_ERROR(get_logger(), "BtNavigator: Invalid status value.");
        throw std::logic_error("BtNavigator::execute: invalid status value");
    }
  }

here:
  RCLCPP_INFO(get_logger(),
    "BtNavigator: Sending the path to the controller to execute.");

  controllerTaskClient_->sendCommand(path);

  // Loop until the control task completes
  for (;; ) {
    // Check to see if this task (navigation) has been canceled. If so, cancel any child
    // tasks and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "BtNavigator: Task has been canceled.");
      controllerTaskClient_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
    }

    // Check if the control task has completed
    auto controlResult = std::make_shared<nav2_tasks::FollowPathResult>();
    TaskStatus status = controllerTaskClient_->waitForResult(controlResult, 10ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        {
          RCLCPP_INFO(get_logger(), "BtNavigator: Control task completed.");
          nav2_tasks::NavigateToPoseResult navigationResult;
          setResult(navigationResult);

          return TaskStatus::SUCCEEDED;
        }

      case TaskStatus::FAILED:
        return TaskStatus::FAILED;

      case TaskStatus::RUNNING:
        RCLCPP_INFO(get_logger(), "BtNavigator: Control task still running.");
        break;

      default:
        RCLCPP_ERROR(get_logger(), "BtNavigator: Invalid status value.");
        throw std::logic_error("BtNavigator::execute: invalid status value");
    }
  }
}

}  // namespace nav2_bt_navigator
