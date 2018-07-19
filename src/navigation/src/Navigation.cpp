// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "navigation/Navigation.hpp"

#include <stdio.h>

Navigation::Navigation()
: Node("navigation"),
  stateMachine_(this)
  // ,missionPlan_(nullptr)
{
  RCLCPP_INFO(get_logger(), "Navigation::Navigation");

  stateMachine_.run();

  cmdSub_ = create_subscription<std_msgs::msg::String>("NavigationCmd",
      std::bind(&Navigation::onCmdReceived, this, std::placeholders::_1));
}

Navigation::~Navigation()
{
  RCLCPP_INFO(get_logger(), "Navigation::~Navigation");
  stateMachine_.halt();
}

void
Navigation::executeNavigationTask(/*const NavigationTask * missionPlan*/)
{
  RCLCPP_INFO(get_logger(), "Navigation::executeNavigationTask");

  // We've been given a task plan to execute. Do any preparatory work
  // and start things off by firing a state transition, which will land
  // us in the Executing state.

  // TODO(mjeronimo): Validate the task
  // missionPlan_ = missionPlan;

  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_EXECUTE_TASK);
}

void
Navigation::cancelNavigationTask()
{
  RCLCPP_INFO(get_logger(), "Navigation::cancelNavigationTask");

  // We've been told to cancel the currently running mission, so fire the
  // state transition to cause a transition to the Canceling state

  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_CANCEL_TASK);
}

void
Navigation::doReadyState()
{
  RCLCPP_INFO(get_logger(), "Navigation::doReadyState");

  // TODO(mjeronimo): Anything we can do while we're waiting for work to do?
}

void
Navigation::doExecutingState()
{
  RCLCPP_INFO(get_logger(), "Navigation::doExecutingState");

  // TODO(mjeronimo): Perform normal processing loop for the mission plan

  // Initiate the state transition

  //  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_TASK_EXECUTED);
  //  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_TASK_FAILED);
  //  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_EXECUTE_TASK_RECOVERY);
}

void
Navigation::doCancelingState()
{
  RCLCPP_INFO(get_logger(), "Navigation::doCancelingState");

  // TODO(mjeronimo): Cancel the currently running mission

  // Initiate the state transition
  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_TASK_CANCELED);
}

void
Navigation::doRecoveringState()
{
  RCLCPP_INFO(get_logger(), "Navigation::doRecoveringState");

  // TODO(mjeronimo): Attempt to perform local recovery

  // Initiate the state transition
  //  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_TASK_RECOVERY_FAILED);
  //  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_TASK_RECOVERY_SUCCEEDED);
}

void
Navigation::doAbortingState()
{
  RCLCPP_INFO(get_logger(), "Navigation::doAbortingState");

  // TODO(mjeronimo): Do the work to abort the current task

  // Initiate the state transition
  stateMachine_.fireEvent(TaskExecutionStateMachine::EVENT_TASK_FAILED);
}

void
Navigation::onCmdReceived(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Navigation::onCmdReceived: \"%s\"", msg->data.c_str())

  if (msg->data.compare("ExecuteNavigationTask") == 0) {
    // NavigationTask *task;
    executeNavigationTask(/*&missionPlan*/);
  } else if (msg->data.compare("CancelNavigationTask") == 0) {
    cancelNavigationTask();
  } else {
    RCLCPP_INFO(get_logger(), "Navigation::onCmdReceived: invalid command: \"%s\"",
      msg->data.c_str())
  }
}
