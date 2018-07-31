// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "planning/AStarPlanner.hpp"
#include <chrono>

AStarPlanner::AStarPlanner(const std::string & name)
: PlanningTaskServer(name)
{
  RCLCPP_INFO(get_logger(), "AStarPlanner::AStarPlanner");
}

AStarPlanner::~AStarPlanner()
{
  RCLCPP_INFO(get_logger(), "AStarPlanner::~AStarPlanner");
}

TaskStatus
AStarPlanner::execute(const nav2_msgs::msg::PathEndPoints::SharedPtr endpoints)
{
  RCLCPP_INFO(get_logger(), "AStarPlanner::execute");

  // Fake out some work
  for (int i = 0; i < 10; i++) {
    RCLCPP_INFO(get_logger(), "AStarPlanner::execute: doing work: %d", i);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // While we're doing the work, check if we've been preempted/canceled
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "AStarPlanner::execute: task has been canceled");
      setCanceled();
      return TaskStatus::CANCELED;
    }
  }

  RCLCPP_INFO(get_logger(), "AStarPlanner::execute: task completed");

  nav2_msgs::msg::Path path;
  setResult(path);

  return TaskStatus::SUCCEEDED;

}
