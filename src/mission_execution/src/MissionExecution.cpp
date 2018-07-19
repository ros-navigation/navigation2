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

void
MissionExecution::executeMission(const MissionPlan * missionPlan)
{
  RCLCPP_INFO(get_logger(), "MissionExecution::executeMission");

  // We've been given a mission plan to execute. Do an preparatory work
  // and start things off by firing a state transition, which will land
  // us in the Executing state. 

  // TODO(mjeronimo): Validate the mission plan for syntax and semantics
  missionPlan_ = missionPlan;

  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_EXECUTE_TASK);
}

void
MissionExecution::cancelMission()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::cancelMission");

  // We've been told to cancel the currently running mission, so fire the 
  // state transition to cause a transition to the Canceling state

  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_CANCEL_TASK);
}

void
MissionExecution::doReadyState()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::doReadyState");

  // TODO(mjeronimo): Anything we can do while we're waiting for work to do?
}

void
MissionExecution::doExecutingState()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::doExecutingState");

  // TODO(mjeronimo): Perform normal processing loop for the mission plan

  // Initiate the state transition

  //  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_TASK_EXECUTED);
  //  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_TASK_FAILED);
  //  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_EXECUTE_TASK_RECOVERY);
}

void
MissionExecution::doCancelingState()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::doCancelingState");

  // TODO(mjeronimo): Cancel the currently running mission

  // Initiate the state transition
  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_TASK_CANCELED);
}

void
MissionExecution::doRecoveringState()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::doRecoveringState");

  // TODO(mjeronimo): Attempt to perform local recovery

  // Initiate the state transition
  //  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_TASK_RECOVERY_FAILED);
  //  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_TASK_RECOVERY_SUCCEEDED);
}

void
MissionExecution::doAbortingState()
{
  RCLCPP_INFO(get_logger(), "MissionExecution::doAbortingState");

  // TODO(mjeronimo): Do the work to abort the current mission

  // Initiate the state transition
  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_TASK_FAILED);
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
