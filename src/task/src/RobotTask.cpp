// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "task/RobotTask.hpp"

RobotTask::RobotTask(const std::string & name, Robot * robot)
: TaskServer(name)
{
  RCLCPP_INFO(get_logger(), "RobotTask::RobotTask");
  robot = robot;
}

RobotTask::~RobotTask()
{
  RCLCPP_INFO(get_logger(), "RobotTask::~RobotTask");
}
