// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "navigation/NavigateToPoseTask.hpp"

NavigateToPoseTask::NavigateToPoseTask(const std::string & name, Robot * robot)
: RobotTask(name, robot)
{
  RCLCPP_INFO(get_logger(), "NavigateToPoseTask::NavigateToPoseTask");
}

NavigateToPoseTask::~NavigateToPoseTask()
{
  RCLCPP_INFO(get_logger(), "NavigateToPoseTask::~NavigateToPoseTask");
}
