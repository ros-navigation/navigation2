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

#include "nav2_simple_navigator/simple_navigator.hpp"

#include <string>
#include <memory>
#include <exception>
#include <chrono>

using namespace std::chrono_literals;
using nav2_tasks::TaskStatus;

namespace nav2_simple_navigator
{

SimpleNavigator::SimpleNavigator()
: Node("SimpleNavigator")
{
  RCLCPP_INFO(get_logger(), "Initializing");

  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](auto) {});

  planner_client_ =
    std::make_unique<nav2_tasks::ComputePathToPoseTaskClient>(temp_node);

  controller_client_ = std::make_unique<nav2_tasks::FollowPathTaskClient>(temp_node);

  task_server_ = std::make_unique<nav2_tasks::NavigateToPoseTaskServer>(temp_node);

  task_server_->setExecuteCallback(
    std::bind(&SimpleNavigator::navigateToPose, this, std::placeholders::_1));
}

SimpleNavigator::~SimpleNavigator()
{
  RCLCPP_INFO(get_logger(), "Shutting down");
}

TaskStatus
SimpleNavigator::navigateToPose(const nav2_tasks::NavigateToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "Begin navigating from current location to (%.2f, %.2f)",
    command->pose.position.x, command->pose.position.y);

  // Create the path to be returned from ComputePath and sent to the FollowPath task
  auto path = std::make_shared<nav2_tasks::ComputePathToPoseResult>();

  RCLCPP_DEBUG(get_logger(), "Getting the path from the planner");
  RCLCPP_DEBUG(get_logger(), "goal->pose.position.x: %f", command->pose.position.x);
  RCLCPP_DEBUG(get_logger(), "goal->pose.position.y: %f", command->pose.position.y);
  RCLCPP_DEBUG(get_logger(), "goal->pose.position.z: %f", command->pose.position.z);
  RCLCPP_DEBUG(get_logger(), "goal->pose.orientation.x: %f", command->pose.orientation.x);
  RCLCPP_DEBUG(get_logger(), "goal->pose.orientation.y: %f", command->pose.orientation.y);
  RCLCPP_DEBUG(get_logger(), "goal->pose.orientation.z: %f", command->pose.orientation.z);
  RCLCPP_DEBUG(get_logger(), "goal->pose.orientation.w: %f", command->pose.orientation.w);

  planner_client_->sendCommand(command);

  // Loop until the subtasks are completed
  for (;; ) {
    // Check to see if this task (navigation) has been canceled. If so, cancel any child
    // tasks and then cancel this task
    if (task_server_->cancelRequested()) {
      RCLCPP_INFO(get_logger(), "Navigation task has been canceled.");
      planner_client_->cancel();
      task_server_->setCanceled();
      return TaskStatus::CANCELED;
    }

    // Check if the planning task has completed
    TaskStatus status = planner_client_->waitForResult(path, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Achieved navigation goal of (%.2f, %.2f)",
          command->pose.position.x, command->pose.position.y);
        goto planning_succeeded;

      case TaskStatus::FAILED:
        RCLCPP_ERROR(get_logger(), "Planning task failed.");
        return TaskStatus::FAILED;

      case TaskStatus::CANCELED:
        RCLCPP_INFO(get_logger(), "Planning task canceled");
        break;

      case TaskStatus::RUNNING:
        RCLCPP_DEBUG(get_logger(), "Planning task still running.");
        break;

      default:
        RCLCPP_ERROR(get_logger(), "Invalid status value.");
        throw std::logic_error("Invalid status value");
    }
  }

planning_succeeded:

  RCLCPP_INFO(get_logger(), "Received path of size %u from planner", path->poses.size());

  int index = 0;
  for (auto pose : path->poses) {
    RCLCPP_DEBUG(get_logger(), "point %u x: %0.2f, y: %0.2f",
      index, pose.position.x, pose.position.y);
    index++;
  }

  RCLCPP_INFO(get_logger(), "Sending path to the controller to execute.");

  controller_client_->sendCommand(path);

  // Loop until the control task completes
  for (;; ) {
    // Check to see if this task (navigation) has been canceled. If so, cancel any child
    // tasks and then cancel this task
    if (task_server_->cancelRequested()) {
      RCLCPP_INFO(get_logger(), "Navigation task has been canceled.");
      controller_client_->cancel();
      task_server_->setCanceled();
      return TaskStatus::CANCELED;
    }

    // Check if the control task has completed
    auto controlResult = std::make_shared<nav2_tasks::FollowPathResult>();
    TaskStatus status = controller_client_->waitForResult(controlResult, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        {
          RCLCPP_INFO(get_logger(), "Control task completed.");

          // This is an empty message, so there are no fields to set
          nav2_tasks::NavigateToPoseResult navigationResult;

          task_server_->setResult(navigationResult);
          return TaskStatus::SUCCEEDED;
        }

      case TaskStatus::FAILED:
        RCLCPP_ERROR(get_logger(), "Control task failed.");
        return TaskStatus::FAILED;

      case TaskStatus::CANCELED:
        RCLCPP_INFO(get_logger(), "Control task canceled");
        break;

      case TaskStatus::RUNNING:
        RCLCPP_DEBUG(get_logger(), "Control task still running");
        break;

      default:
        RCLCPP_ERROR(get_logger(), "Invalid status value.");
        throw std::logic_error("Invalid status value");
    }
  }
}

}  // namespace nav2_simple_navigator
