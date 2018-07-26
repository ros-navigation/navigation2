// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "planning/AStarPlanner.hpp"
#include <chrono>

AStarPlanner::AStarPlanner(const std::string & name)
: PlanningTask(name)
{
  RCLCPP_INFO(get_logger(), "AStarPlanner::AStarPlanner");
}

AStarPlanner::~AStarPlanner()
{
  RCLCPP_INFO(get_logger(), "AStarPlanner::~AStarPlanner");
}

void
AStarPlanner::createPlan(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  RCLCPP_INFO(get_logger(), "AStarPlanner::createPlan");
}

void
AStarPlanner::workerThread()
{
  RCLCPP_INFO(get_logger(), "AStarPlanner::workerThread");

  while (!stopWorkerThread_)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(get_logger(), "AStarPlanner::workerThread: doing work");
  }
}

