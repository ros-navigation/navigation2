// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef ROBOT__ROSROBOT_HPP_
#define ROBOT__ROSROBOT_HPP_

#include <string>
#include "robot/Robot.hpp"

class RosRobot : public Robot
{
public:
  /**
   * Construct a RosRobot with a provided URDF file.
   *
   * @param[in] filename The filename of the URDF file describing this robot.
   */
  explicit RosRobot(std::string & filename);

  RosRobot() = delete;
  ~RosRobot();

  void enterSafeState() override;
};

#endif  // ROBOT__ROSROBOT_HPP_
