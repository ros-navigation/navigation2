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
  RCLCPP_INFO(get_logger(), "Starting node");

  plannerTaskClient_ = std::make_unique<nav2_tasks::ComputePathToPoseTaskClient>(this);

  if (!plannerTaskClient_->waitForServer(nav2_tasks::defaultServerTimeout)) {
    RCLCPP_ERROR(get_logger(), "Planner not running");
    throw std::runtime_error("Planner not running");
  }

  controllerTaskClient_ = std::make_unique<nav2_tasks::FollowPathTaskClient>(this);

  if (!controllerTaskClient_->waitForServer(nav2_tasks::defaultServerTimeout)) {
    RCLCPP_ERROR(get_logger(), "Controller not running");
    throw std::runtime_error("Controller not running");
  }
}

SimpleNavigator::~SimpleNavigator()
{
  RCLCPP_INFO(get_logger(), "Stopping node");
}

TaskStatus
SimpleNavigator::execute(const nav2_tasks::NavigateToPoseCommand::SharedPtr command)
{
  RCLCPP_DEBUG(get_logger(), "Execute navigate to pose");

  // Compose the PathEndPoints message for Navigation. The starting pose comes from
  // localization, while the goal pose is from the incoming command
  auto endpoints = std::make_shared<nav2_tasks::ComputePathToPoseCommand>();

  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose;
  if (robot_.getCurrentPose(current_pose)) {
    RCLCPP_DEBUG(get_logger(), "got robot pose");
    endpoints->start = current_pose->pose.pose;
    endpoints->goal = command->pose;
    endpoints->tolerance = 2.0;
  } else {
    // TODO(mhpanah): use either last known pose, current pose from odom, wait, or try again.
    RCLCPP_ERROR(get_logger(), "Current robot pose is not available");
    return TaskStatus::FAILED;
  }

  RCLCPP_DEBUG(get_logger(), "Getting the path from the planner");
  auto path = std::make_shared<nav2_tasks::ComputePathToPoseResult>();
  plannerTaskClient_->sendCommand(endpoints);

  // Loop until the subtasks are completed
  for (;; ) {
    // Check to see if this task (navigation) has been canceled. If so, cancel any child
    // tasks and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "Task has been canceled");
      plannerTaskClient_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
    }

    // Check if the planning task has completed
    TaskStatus status = plannerTaskClient_->waitForResult(path, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Planning task completed");
        goto planning_succeeded;

      case TaskStatus::FAILED:
        RCLCPP_ERROR(get_logger(), "Planning task failed");
        return TaskStatus::FAILED;

      case TaskStatus::RUNNING:
        RCLCPP_DEBUG(get_logger(), "Planning task still running");
        break;

      default:
        RCLCPP_ERROR(get_logger(), "Invalid status value");
        throw std::logic_error("Invalid status value");
    }
  }

planning_succeeded:

  RCLCPP_DEBUG(get_logger(), "Got path of size %u", path->poses.size());

  int index = 0;
  for (auto pose : path->poses) {
    RCLCPP_DEBUG(get_logger(), "point %u x: %0.2f, y: %0.2f",
      index, pose.position.x, pose.position.y);
    index++;
  }

  RCLCPP_INFO(get_logger(), "Sending the path to the controller to execute");

  controllerTaskClient_->sendCommand(path);

  // Loop until the control task completes
  for (;; ) {
    // Check to see if this task (navigation) has been canceled. If so, cancel any child
    // tasks and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "Task has been canceled");
      controllerTaskClient_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
    }

    // Check if the control task has completed
    auto controlResult = std::make_shared<nav2_tasks::FollowPathResult>();
    TaskStatus status = controllerTaskClient_->waitForResult(controlResult, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        {
          RCLCPP_INFO(get_logger(), "Control task completed");
          nav2_tasks::NavigateToPoseResult navigationResult;
          setResult(navigationResult);
          return TaskStatus::SUCCEEDED;
        }

      case TaskStatus::FAILED:
        RCLCPP_ERROR(get_logger(), "Control task failed");
        return TaskStatus::FAILED;

      case TaskStatus::RUNNING:
        RCLCPP_DEBUG(get_logger(), "Control task still running");
        break;

      default:
        RCLCPP_ERROR(get_logger(), "Invalid status value");
        throw std::logic_error("Invalid status value");
    }
  }
}

}  // namespace nav2_simple_navigator
