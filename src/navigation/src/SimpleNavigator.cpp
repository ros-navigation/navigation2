// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "navigation/SimpleNavigator.hpp"
#include <chrono>

SimpleNavigator::SimpleNavigator(const std::string & name, Robot * robot)
: NavigateToPoseTask(name, robot)
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::SimpleNavigator");

// TODO: make into C++ smart pointers
  planner_ = new TaskClient("AStarPlanner", this);
  controller_ = new TaskClient("DwaController", this);
}

SimpleNavigator::~SimpleNavigator()
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::~SimpleNavigator");
}

void
SimpleNavigator::navigateTo()
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::executePlan");
}

void
SimpleNavigator::workerThread()
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::workerThread");

  RCLCPP_INFO(get_logger(), "SimpleNavigator::workerThread: sending executes");
  planner_->execute();
  controller_->execute();

  while (!stopWorkerThread_ /* && rclcpp::ok() */ )
  {
    RCLCPP_INFO(get_logger(), "SimpleNavigator::workerThread: doing work");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  RCLCPP_INFO(get_logger(), "SimpleNavigator::workerThread: sending cancels");
  planner_->cancel();
  controller_->cancel();
}
