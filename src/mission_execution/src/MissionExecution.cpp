// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "mission_execution/MissionExecution.hpp"

MissionExecution::MissionExecution()
: TaskServer("mission_execution")
  //missionPlan_(nullptr)
{
  RCLCPP_INFO(get_logger(), "MissionExecution::MissionExecution");

  // TODO: make into C++ smart pointer
  navigateToPoseTask_ = new TaskClient("SimpleNavigator", this);
}

MissionExecution::~MissionExecution()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::~MissionExecution");
}

void
MissionExecution::execute(/*const MissionPlan & missionPlan*/)
{
  RCLCPP_INFO(get_logger(), "MissionExecution:execute");

  // TODO(mjeronimo): Validate the mission plan for syntax and semantics
  // missionPlan_ = missionPlan;

  navigateToPoseTask_->execute();

  // Simulate looping until navigation reaches a terminal state
  for (int i=0; i<5; i++)
  {
    // success/failure/running = navigateToPoseTask_->waitForResult(timeout)
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    if (isPreemptRequested()) {
      RCLCPP_INFO(get_logger(), "MissionExecution::execute: task has been preempted");
      navigateToPoseTask_->cancel();
	  setPreempted();
      return;
    }
  }

  RCLCPP_INFO(get_logger(), "MissionExecution::execute: task completed");
}
