// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "mission_execution/MissionExecution.hpp"

MissionExecution::MissionExecution(const std::string & name)
: MissionExecutionTaskServer(name)
  // missionPlan_(nullptr)
{
  RCLCPP_INFO(get_logger(), "MissionExecution::MissionExecution");
  navigationTask_ = std::make_unique<NavigateToPoseTaskClient>("SimpleNavigator", this);
}

MissionExecution::~MissionExecution()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::~MissionExecution");
}

TaskStatus
MissionExecution::execute(const std_msgs::msg::String::SharedPtr /*command*/)
{
  RCLCPP_INFO(get_logger(), "MissionExecution:execute");

  // TODO(mjeronimo): Validate the mission plan for syntax and semantics
  // missionPlan_ = missionPlan;

  navigationTask_->execute();

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
        throw("MissionExecution::execute: invalid status value");
    }
  }
}
