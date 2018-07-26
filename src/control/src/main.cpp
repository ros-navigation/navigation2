// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "control/DwaController.hpp"
#include "robot/RosRobot.hpp"

int main(int argc, char ** argv)
{
  std::string urdf_filename;
  RosRobot robot(urdf_filename);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DwaController>("DwaController", &robot));
  rclcpp::shutdown();

  return 0;
}
