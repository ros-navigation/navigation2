// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__ASTARPLANNER_HPP_
#define TASK__ASTARPLANNER_HPP_

#include "task/PlanningTask.hpp"

class AStarPlanner : public PlanningTask
{
public:
  AStarPlanner(const std::string & name);
  AStarPlanner() = delete;
  ~AStarPlanner();

  void createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

protected:
  void workerThread() override;
};

#endif  // TASK__ASTARPLANNER_HPP_
