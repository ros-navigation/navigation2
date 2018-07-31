// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include <memory>
#include <exception>
#include "rclcpp/rclcpp.hpp"
#include "mission_execution/MissionExecution.hpp"

int main(int argc, char ** argv)
{
  try {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionExecution>("MissionExecution"));
    rclcpp::shutdown();
  } catch (std::exception ex) {
    std::cout << ex.what() << std::endl;
  } catch (char const *s) {
    std::cout << s << std::endl;
  }
  return 0;
}
