// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

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
MissionExecution::execute(const nav2_msgs::msg::MissionPlan::SharedPtr missionPlan)
{
  RCLCPP_INFO(get_logger(), "MissionExecution:execute");
  RCLCPP_INFO(get_logger(), "MissionExecution:execute: plan: %s", missionPlan->mission_plan.c_str());

  // TODO(mjeronimo): Validate the mission plan for syntax and semantics

  std_msgs::msg::String::SharedPtr command;
  navigationTask_->execute(command);

  // Simulate looping until navigation reaches a terminal state
  for (;;) {
    // Check to see if this task has been canceled. If so, cancel the navigation
    // task first and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "MissionExecution::execute: task has been canceled");
      navigationTask_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
    }

    // This task hasn't been canceled, so see if the child task has finished
    std_msgs::msg::String navResult;
    TaskStatus status = navigationTask_->waitForResult(navResult, 100);

    switch (status)
    {
      case TaskStatus::SUCCEEDED:
      {
        RCLCPP_INFO(get_logger(), "MissionExecution::execute: navigation task completed");
        RCLCPP_INFO(get_logger(), "MissionExecution::execute: msg: %s", navResult.data.c_str());

        std_msgs::msg::String missionExecutionResult;
        missionExecutionResult.data = "Here is the result from MissionExecution";
        setResult(missionExecutionResult);

        return TaskStatus::SUCCEEDED;
      }

      case TaskStatus::FAILED:
        return TaskStatus::FAILED;

      case TaskStatus::RUNNING:
        RCLCPP_INFO(get_logger(), "MissionExecution::execute: navigation task still running");
        break;

      default:
        RCLCPP_INFO(get_logger(), "MissionExecution::execute: invalid status value");
        throw std::logic_error("MissionExecution::execute: invalid status value");
    }
  }
}
