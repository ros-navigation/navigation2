// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "navigation/SimpleNavigator.hpp"
#include <chrono>

SimpleNavigator::SimpleNavigator(const std::string & name, Robot * robot)
: NavigateToPoseTask(name, robot)
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::SimpleNavigator");

  // TODO(mjeronimo): make into C++ smart pointers
  planner_ = new TaskClient("AStarPlanner", this);
  controller_ = new TaskClient("DwaController", this);
}

SimpleNavigator::~SimpleNavigator()
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::~SimpleNavigator");
}

TaskServer::Status
SimpleNavigator::execute(const CommandMsg::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::execute");

  RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: getting the path from the planner");
  planner_->execute();
  auto planningResult = std::make_shared<std_msgs::msg::String>();

  // Simulate looping until the planner reaches a terminal state
  for (;;) {
    // Check to see if this task has been canceled. If so, cancel any child tasks
	// and bail out
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: task has been canceled");
      planner_->cancel();
      setCanceled();
      return TaskServer::CANCELED;
    }

    // Otherwise, check if the child task has completed (succeeded or failed)
	TaskClient::Status status = planner_->waitForResult(planningResult /*, timeout*/);

	switch (status)
	{
	  case TaskClient::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: planning task completed");
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: msg: %s", planningResult->data);

		goto here;

	  case TaskClient::FAILED:
        return TaskServer::FAILED;

	  case TaskClient::RUNNING:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: planning task still running");
		break;

	  default:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: invalid status value");
        throw("SimpleNavigator::execute: invalid status value");
    }
  }

here:
  RCLCPP_INFO(get_logger(),
      "SimpleNavigator::execute: sending the path to the controller to execute");

  controller_->execute();
  auto controlResult = std::make_shared<std_msgs::msg::String>();

  // Simulate looping until the controller reaches a terminal state
  for (;;) {
    // Check to see if this task has been canceled. If so, cancel any child tasks
	// and bail out
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: task has been canceled");
      controller_->cancel();
      setCanceled();
      return TaskServer::CANCELED;
	}

    // Otherwise, check if the child task has completed (succeeded or failed)
	TaskClient::Status status = controller_->waitForResult(controlResult /*, timeout*/);

	switch (status)
	{
	  case TaskClient::SUCCEEDED:
	  {
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: control task completed");
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: msg: %s", controlResult->data.c_str());
        ResultMsg navigationResult;
        navigationResult.data = "Here is the result from the SimpleNavigator!";
        sendResult(navigationResult);

        return TaskServer::SUCCEEDED;
      }

	  case TaskClient::FAILED:
        return TaskServer::FAILED;

	  case TaskClient::RUNNING:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: control task still running");
		break;

	  default:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: invalid status value");
        throw("SimpleNavigator::execute: invalid status value");
    }
  }
}
