// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__NAVIGATIONTASK_HPP_
#define TASK__NAVIGATIONTASK_HPP_

#include "task/RobotTask.hpp"
#include "task/AStarPlanner.hpp"
#include "task/SpecificController.hpp"

class NavigationTask : public RobotTask
{
public:
  explicit NavigationTask(const std::string & name, Robot * robot);
  NavigationTask() = delete;
  ~NavigationTask();

protected:
  AStarPlanner planner_;
  SpecificController controller_;
};

#endif  // TASK__NAVIGATIONTASK_HPP_
