// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mission_execution/MissionExecution.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExecution>());
  rclcpp::shutdown();

  return 0;
}
