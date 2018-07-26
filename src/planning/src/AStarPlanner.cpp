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

#if 0
void
AStarPlanner::createPlan(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  RCLCPP_INFO(get_logger(), "AStarPlanner::createPlan");
}
#endif

void
AStarPlanner::execute()
{
  RCLCPP_INFO(get_logger(), "AStarPlanner::execute");

  // Fake out some work
  for (int i=0; i<10; i++)
  {
    RCLCPP_INFO(get_logger(), "AStarPlanner::execute: doing work: %d", i);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    // While we're doing the work, check if we've been preempted/canceled
    if (isPreemptRequested()) {
      RCLCPP_INFO(get_logger(), "AStarPlanner::execute: task has been preempted");
	  setPreempted();
      return;
    }
  }

  // TODO: return the result
  RCLCPP_INFO(get_logger(), "AStarPlanner::execute: task completed");
}

