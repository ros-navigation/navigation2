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
#include "nav2_simple_navigator/simple_navigator.hpp"

using namespace std::chrono_literals;
using nav2_tasks::TaskStatus;

namespace nav2_simple_navigator
{

SimpleNavigator::SimpleNavigator()
: nav2_tasks::NavigateToPoseTaskServer("NavigateToPoseNode"),
  robot_(this)
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::SimpleNavigator");

  plannerTaskClient_ = std::make_unique<nav2_tasks::ComputePathToPoseTaskClient>(this);

  if (!plannerTaskClient_->waitForServer(nav2_tasks::defaultServerTimeout)) {
    RCLCPP_ERROR(get_logger(), "SimpleNavigator: planner not running");
    throw std::runtime_error("SimpleNavigator: planner not running");
  }

  controllerTaskClient_ = std::make_unique<nav2_tasks::FollowPathTaskClient>(this);

  if (!controllerTaskClient_->waitForServer(nav2_tasks::defaultServerTimeout)) {
    RCLCPP_ERROR(get_logger(), "SimpleNavigator: controller not running");
    throw std::runtime_error("SimpleNavigator: controller not running");
  }
}

SimpleNavigator::~SimpleNavigator()
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::~SimpleNavigator");
}

TaskStatus
SimpleNavigator::execute(const nav2_tasks::NavigateToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::execute");

  // Compose the PathEndPoints message for Navigation. The starting pose comes from 
  // localization, while the goal pose is from the incoming command
  auto endpoints = std::make_shared<nav2_tasks::ComputePathToPoseCommand>();

  endpoints->start = robot_.getCurrentPose()->pose.pose;
  endpoints->goal = command->pose;
  endpoints->tolerance = 2.0;

  RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: getting the path from the planner");
  auto path = std::make_shared<nav2_tasks::ComputePathToPoseResult>();
  plannerTaskClient_->sendCommand(endpoints);

  // Loop until the subtasks are completed
  for (;; ) {
    // Check to see if this task (navigation) has been canceled. If so, cancel any child
    // tasks and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: task has been canceled");
      plannerTaskClient_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
    }

    // Check if the planning task has completed
    TaskStatus status = plannerTaskClient_->waitForResult(path, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: planning task completed");
        goto planning_succeeded;

      case TaskStatus::FAILED:
        RCLCPP_ERROR(get_logger(), "SimpleNavigator::execute: planning task failed");
        return TaskStatus::FAILED;

      case TaskStatus::RUNNING:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: planning task still running");
        break;

      default:
        RCLCPP_ERROR(get_logger(), "SimpleNavigator::execute: invalid status value");
        throw std::logic_error("SimpleNavigator::execute: invalid status value");
    }
  }

planning_succeeded:

  RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: got path of size %u",
    path->poses.size());

  int index = 0;
  for (auto pose : path->poses) {
    RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: point %u x: %0.2f, y: %0.2f",
      index, pose.position.x, pose.position.y);
    index++;
  }

  RCLCPP_INFO(get_logger(),
    "SimpleNavigator::execute: sending the path to the controller to execute");

  controllerTaskClient_->sendCommand(path);

  // Loop until the control task completes
  for (;; ) {
    // Check to see if this task (navigation) has been canceled. If so, cancel any child
    // tasks and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: task has been canceled");
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
          RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: control task completed");
          nav2_tasks::NavigateToPoseResult navigationResult;
          setResult(navigationResult);

          return TaskStatus::SUCCEEDED;
        }

      case TaskStatus::FAILED:
        RCLCPP_ERROR(get_logger(), "SimpleNavigator::execute: control task failed");
        return TaskStatus::FAILED;

      case TaskStatus::RUNNING:
        break;

      default:
        RCLCPP_ERROR(get_logger(), "SimpleNavigator::execute: invalid status value");
        throw std::logic_error("SimpleNavigator::execute: invalid status value");
    }
  }
}

}  // namespace nav2_simple_navigator
