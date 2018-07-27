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

  // Simulate looping until navigation reaches a terminal state
  for (int i = 0; i < 5; i++) {
    // auto result = navigateToPoseTask_->waitForResult(timeout)
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "MissionExecution::execute: task has been canceled");
      navigateToPoseTask_->cancel();
      setCanceled();
      return TaskServer::CANCELED;
    }
  }

  RCLCPP_INFO(get_logger(), "MissionExecution::execute: task completed");

  ResultMsg result;
  result.data = "Here is the result from MissionExecution";
  sendResult(result);

  return TaskServer::SUCCEEDED;
}
