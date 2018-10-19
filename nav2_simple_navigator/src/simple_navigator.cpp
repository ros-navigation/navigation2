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
  RCLCPP_INFO(get_logger(), "Initializing SimpleNavigator");

  plannerTaskClient_ = std::make_unique<nav2_tasks::ComputePathToPoseTaskClient>(this);

  if (!plannerTaskClient_->waitForServer(nav2_tasks::defaultServerTimeout)) {
    RCLCPP_ERROR(get_logger(), "SimpleNavigator: global planner is not running");
    throw std::runtime_error("SimpleNavigator: planner not running");
  }

  controllerTaskClient_ = std::make_unique<nav2_tasks::FollowPathTaskClient>(this);

  if (!controllerTaskClient_->waitForServer(nav2_tasks::defaultServerTimeout)) {
    RCLCPP_ERROR(get_logger(), "SimpleNavigator: controller is not running");
    throw std::runtime_error("SimpleNavigator: controller not running");
  }
}

SimpleNavigator::~SimpleNavigator()
{
  RCLCPP_DEBUG(get_logger(), "Stopping node");
}

TaskStatus
SimpleNavigator::execute(const nav2_tasks::NavigateToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator: Executing new command to (%.2f, %.2f)",
    command->pose.position.x, command->pose.position.y);

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
    //TODO(mhpanah): use either last known pose, current pose from odom, wait, or try again.
    RCLCPP_WARN(get_logger(), "Current Robot Pose is not available.");
    return TaskStatus::FAILED;
  }

  RCLCPP_INFO(get_logger(), "SimpleNavigator: Requesting path from the planner server.");
  auto path = std::make_shared<nav2_tasks::ComputePathToPoseResult>();
  plannerTaskClient_->sendCommand(endpoints);

  // Loop until the subtasks are completed
  for (;; ) {
    // Check to see if this task (navigation) has been canceled. If so, cancel any child
    // tasks and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "SimpleNavigator: Navigation task has been canceled.");
      plannerTaskClient_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
    }

    // Check if the planning task has completed
    TaskStatus status = plannerTaskClient_->waitForResult(path, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "SimpleNavigator: Achieved navigation goal of (%.2f, %.2f)",
          command->pose.position.x, command->pose.position.y);
        goto planning_succeeded;

      case TaskStatus::FAILED:
        RCLCPP_ERROR(get_logger(), "SimpleNavigator: Planning task failed.");
        return TaskStatus::FAILED;

      case TaskStatus::RUNNING:
        RCLCPP_DEBUG(get_logger(), "SimpleNavigator: Planning task still running.");
        break;

      default:
        RCLCPP_ERROR(get_logger(), "SimpleNavigator: Invalid status value.");
        throw std::logic_error("SimpleNavigator::execute: invalid status value");
    }
  }

planning_succeeded:

  RCLCPP_INFO(get_logger(), "SimpleNavigator: Received path of size %u from planner",
    path->poses.size());

  int index = 0;
  for (auto pose : path->poses) {
    RCLCPP_DEBUG(get_logger(), "point %u x: %0.2f, y: %0.2f",
      index, pose.position.x, pose.position.y);
    index++;
  }

  RCLCPP_INFO(get_logger(), "SimpleNavigator: Sending path to the controller to execute.");

  controllerTaskClient_->sendCommand(path);

  // Loop until the control task completes
  for (;; ) {
    // Check to see if this task (navigation) has been canceled. If so, cancel any child
    // tasks and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "SimpleNavigator: Navigation task has been canceled.");
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
          RCLCPP_INFO(get_logger(), "SimpleNavigator: Control task completed.");
          nav2_tasks::NavigateToPoseResult navigationResult;
          setResult(navigationResult);
          return TaskStatus::SUCCEEDED;
        }

      case TaskStatus::FAILED:
        RCLCPP_ERROR(get_logger(), "SimpleNavigator: Control task failed.");
        return TaskStatus::FAILED;

      case TaskStatus::RUNNING:
        RCLCPP_DEBUG(get_logger(), "Control task still running");
        break;

      default:
        RCLCPP_ERROR(get_logger(), "SimpleNavigator: Invalid status value.");
        throw std::logic_error("SimpleNavigator::execute: invalid status value");
    }
  }
}

}  // namespace nav2_simple_navigator
