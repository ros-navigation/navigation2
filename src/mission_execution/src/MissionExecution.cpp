// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "mission_execution/MissionExecution.hpp"

#include <stdio.h>

MissionExecution::MissionExecution()
: Node("mission_execution"),
  missionPlan_(nullptr)
{
  RCLCPP_INFO(get_logger(), "MissionExecution::MissionExecution");

  cmdSub_ = create_subscription<std_msgs::msg::String>("MissionExecutionCmd",
      std::bind(&MissionExecution::onCmdReceived, this, std::placeholders::_1));
}

MissionExecution::~MissionExecution()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::~MissionExecution");
}

void
MissionExecution::executeMission(const MissionPlan * missionPlan)
{
  RCLCPP_INFO(get_logger(), "MissionExecution::executeMission");

  // We've been given a mission plan to execute. Do an preparatory work
  // and start things off by firing a state transition, which will land
  // us in the Executing state.

  // TODO(mjeronimo): Validate the mission plan for syntax and semantics
  missionPlan_ = missionPlan;
}

void
MissionExecution::cancelMission()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::cancelMission");

  // We've been told to cancel the currently running mission, so fire the
  // state transition to cause a transition to the Canceling state
}

void
MissionExecution::onCmdReceived(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "MissionExecution::onCmdReceived: \"%s\"", msg->data.c_str())

  if (msg->data.compare("ExecuteMission") == 0) {
    MissionPlan missionPlan;
    executeMission(&missionPlan);
  } else if (msg->data.compare("CancelMission") == 0) {
    cancelMission();
  } else {
    RCLCPP_INFO(get_logger(), "MissionExecution::onCmdReceived: invalid command: \"%s\"",
      msg->data.c_str())
  }
}
