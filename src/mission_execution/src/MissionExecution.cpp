// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "mission_execution/MissionExecution.hpp"

#include <stdio.h>

MissionExecution::MissionExecution()
: Node("mission_execution"),
  stateMachine_(this),
  missionPlan_(nullptr)
{
  RCLCPP_INFO(get_logger(), "MissionExecution::MissionExecution");

  stateMachine_.run();

  cmdSub_ = create_subscription<std_msgs::msg::String>("MissionExecutionCmd",
      std::bind(&MissionExecution::onCmdReceived, this, std::placeholders::_1));
}

MissionExecution::~MissionExecution()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::~MissionExecution");
  stateMachine_.halt();
}

void MissionExecution::executeMission(const MissionPlan * missionPlan)
{
  RCLCPP_INFO(get_logger(), "MissionExecution::executeMission");

  missionPlan_ = missionPlan;
  stateMachine_.fireEvent(MissionExecutionStateMachine::EVENT_EXECUTE_MISSION);
}

void MissionExecution::cancelMission()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::cancelMission");
  stateMachine_.fireEvent(MissionExecutionStateMachine::EVENT_CANCEL_MISSION);
}

void MissionExecution::doReadyState()
{
  // This is the starting state. Do any work to reset operations.

  RCLCPP_INFO(get_logger(), "MissionExecution::doReadyState");
}

void MissionExecution::doExecutingState()
{
  // TODO(mjeronimo): Validate input for syntax and semantics

  // TODO(mjeronimo): Perform normal processing loop for the mission plan

  // Initiate the state transition
  //  stateMachine_.fireEvent(MissionExecutionStateMachine::EVENT_MISSION_EXECUTED);
  // Or:
  //  stateMachine_.fireEvent(MissionExecutionStateMachine::EVENT_MISSION_FAILED);
  // Or:
  //  stateMachine_.fireEvent(MissionExecutionStateMachine::EVENT_EXECUTE_RECOVERY);

  RCLCPP_INFO(get_logger(), "MissionExecution::doExecutingState");
}

void MissionExecution::doCancelingState()
{
  // TODO(mjeronimo): Cancel the currently running mission

  RCLCPP_INFO(get_logger(), "MissionExecution::doCancelingState");
}

void MissionExecution::doRecoveringState()
{
  // TODO(mjeronimo): Attempt to perform local recovery

  RCLCPP_INFO(get_logger(), "MissionExecution::doRecoveringState");
}

void MissionExecution::doAbortingState()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::doAbortingState");

  // TODO(mjeronimo): Do the work to abort the current mission

  // Initiate the state transition
  stateMachine_.fireEvent(MissionExecutionStateMachine::EVENT_MISSION_FAILED);
}

void MissionExecution::onCmdReceived(const std_msgs::msg::String::SharedPtr msg)
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
