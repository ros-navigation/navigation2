// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef ROBOT__ROBOT_HPP_
#define ROBOT__ROBOT_HPP_

class Robot
{
public:
  virtual ~Robot() {}

  // Commands to the robot
  virtual void enterSafeState() = 0;
};

#endif  // ROBOT__ROBOT_HPP_
