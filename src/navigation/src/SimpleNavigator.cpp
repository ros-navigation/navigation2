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

void
SimpleNavigator::execute()
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::execute");

  RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: getting the path from the planner");
  planner_->execute();

  // Simulate looping until the planner reaches a terminal state
  for (int i = 0; i < 5; i++) {
    // success/failure/running = planner->waitForResult(timeout)
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: task has been canceled");
      planner_->cancel();
      setCanceled();
      return;
    }
  }

  RCLCPP_INFO(
    get_logger(), "SimpleNavigator::execute: sending the path to the controller to execute");
  controller_->execute();

  // Simulate looping until the controller reaches a terminal state
  for (int i = 0; i < 5; i++) {
    // success/failure/running = controller->waitForResult()
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: task has been canceled");
      controller_->cancel();
      setCanceled();
      return;
    }
  }

  RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: task completed");
}
