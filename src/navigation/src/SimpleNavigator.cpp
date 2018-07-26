// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "navigation/SimpleNavigator.hpp"

SimpleNavigator::SimpleNavigator(const std::string & name, Robot * robot)
: NavigateToPoseTask(name, robot), planner_(nullptr), controller_(nullptr)
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::SimpleNavigator");
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
}
