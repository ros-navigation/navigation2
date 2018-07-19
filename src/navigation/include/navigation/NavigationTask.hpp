// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef NAVIGATION__NAVIGATIONTASK_HPP_
#define NAVIGATION__NAVIGATIONTASK_HPP_

#include "robot/Robot.hpp"

class NavigationTask
{
public:
  NavigationTask();
  ~NavigationTask();

protected:
  Robot * robot_;
};

#endif  // NAVIGATION__NAVIGATIONTASK_HPP_
