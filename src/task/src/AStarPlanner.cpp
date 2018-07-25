// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "task/AStarPlanner.hpp"

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
}

void
AStarPlanner::workerThread()
{
}
