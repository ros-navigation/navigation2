// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "task/PlanningTask.hpp"

PlanningTask::PlanningTask(const std::string & name)
: Task(name)
{
  RCLCPP_INFO(get_logger(), "PlanningTask::PlanningTask");
}

PlanningTask::~PlanningTask()
{
  RCLCPP_INFO(get_logger(), "PlanningTask::~PlanningTask");
}
