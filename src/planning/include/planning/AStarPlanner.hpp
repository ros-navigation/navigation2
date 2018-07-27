// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef PLANNING__ASTARPLANNER_HPP_
#define PLANNING__ASTARPLANNER_HPP_

#include "planning/PlanningTask.hpp"

class AStarPlanner : public PlanningTask
{
public:
  explicit AStarPlanner(const std::string & name);
  AStarPlanner() = delete;
  ~AStarPlanner();

  // void createPlan(const geometry_msgs::msg::PoseStamped & start,
  //   const geometry_msgs::msg::PoseStamped & goal);

  void execute() override;
};

#endif  // PLANNING__ASTARPLANNER_HPP_
