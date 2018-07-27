// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "mission_execution/MissionExecution.hpp"

MissionExecution::MissionExecution(const std::string & name)
: TaskServer(name)
  // missionPlan_(nullptr)
{
  RCLCPP_INFO(get_logger(), "MissionExecution::MissionExecution");

  // TODO(mjeronimo): make into C++ smart pointer
  navigateToPoseTask_ = new TaskClient("SimpleNavigator", this);
}

MissionExecution::~MissionExecution()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::~MissionExecution");
}

TaskServer::Status
MissionExecution::execute(const CommandMsg::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "MissionExecution:execute");

  // TODO(mjeronimo): Validate the mission plan for syntax and semantics
  // missionPlan_ = missionPlan;

  navigateToPoseTask_->execute();
  auto navResult = std::make_shared<std_msgs::msg::String>();

  // Simulate looping until navigation reaches a terminal state
  for (;;) {
    // Check to see if this task has been canceled. If so, cancel any child tasks
    // and bail out
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "MissionExecution::execute: task has been canceled");
      navigateToPoseTask_->cancel();
      setCanceled();
      return TaskServer::CANCELED;
    }

    TaskClient::Status status = navigateToPoseTask_->waitForResult(navResult);

    switch (status)
    {
      case TaskClient::SUCCEEDED:
      {
        RCLCPP_INFO(get_logger(), "MissionExecution::execute: navigation task completed");
        RCLCPP_INFO(get_logger(), "MissionExecution::execute: msg: %s", navResult->data.c_str());

        ResultMsg missionExecutionResult;
        missionExecutionResult.data = "Here is the result from MissionExecution";
        sendResult(missionExecutionResult);

        return TaskServer::SUCCEEDED;
      }

      case TaskClient::FAILED:
        return TaskServer::FAILED;

      case TaskClient::RUNNING:
        RCLCPP_INFO(get_logger(), "MissionExecution::execute: navigation task still running");
        break;

      default:
        RCLCPP_INFO(get_logger(), "MissionExecution::execute: invalid status value");
        throw("MissionExecution::execute: invalid status value");
    }
  }
}
