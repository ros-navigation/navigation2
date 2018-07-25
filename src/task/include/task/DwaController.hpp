// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__SPECIFICCONTROLLER_HPP_
#define TASK__SPECIFICCONTROLLER_HPP_

#include "task/ControlTask.hpp"

class DwaController : public ControlTask
{
public:
  DwaController(const std::string & name, Robot * robot);
  DwaController() = delete;
  ~DwaController();

  void executePlan() override; 

protected:
  void workerThread() override; 
};

#endif  // TASK__SPECIFICCONTROLLER_HPP_
