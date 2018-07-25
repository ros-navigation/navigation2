// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "task/NavigationTask.hpp"

NavigationTask::NavigationTask(const std::string & name, Robot * robot)
: RobotTask(name, robot), planner_(name + "Planner"), controller_(name + "Controller", robot)
{
  RCLCPP_INFO(get_logger(), "NavigationTask::NavigationTask");
}

NavigationTask::~NavigationTask()
{
  RCLCPP_INFO(get_logger(), "NavigationTask::~NavigationTask");
}
