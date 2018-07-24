// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__ROBOTTASK_HPP_
#define TASK__ROBOTTASK_HPP_

#include "task/Task.hpp"
#include "robot/Robot.hpp"

class RobotTask : public Task
{
public:
  explicit RobotTask(Robot * robot);
  RobotTask() = delete;
  ~RobotTask();

protected:
  Robot * robot_;
};

#endif  // TASK__ROBOTTASK_HPP_
