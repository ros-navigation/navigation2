// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef CONTROL__CONTROLTASK_HPP_
#define CONTROL__CONTROLTASK_HPP_

#include "task/RobotTask.hpp"

class ControlTask : public RobotTask
{
public:
  ControlTask(const std::string & name, Robot * robot);
  ControlTask() = delete;
  ~ControlTask();

  virtual void executePlan() = 0;
};

#endif  // CONTROL__CONTROLTASK_HPP_
