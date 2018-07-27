// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef NAVIGATION__NAVIGATETOPOSETASK_HPP_
#define NAVIGATION__NAVIGATETOPOSETASK_HPP_

#include "task/RobotTask.hpp"

class NavigateToPoseTask : public RobotTask
{
public:
  NavigateToPoseTask(const std::string & name, Robot * robot);
  NavigateToPoseTask() = delete;
  ~NavigateToPoseTask();

  // virtual void navigateTo(/*pose*/) = 0;
};

#endif  // NAVIGATION__NAVIGATETOPOSETASK_HPP_
