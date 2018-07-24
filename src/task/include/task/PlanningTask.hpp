// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__PLANNINGTASK_HPP_
#define TASK__PLANNINGTASK_HPP_

#include "task/RobotTask.hpp"

class PlanningTask : public RobotTask
{
public:
  explicit PlanningTask(Robot * robot);
  PlanningTask() = delete;
  ~PlanningTask();

  // getPlan()
};

#endif  // TASK__PLANNINGTASK_HPP_
