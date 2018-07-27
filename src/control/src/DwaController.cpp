// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "control/DwaController.hpp"
#include <chrono>

DwaController::DwaController(const std::string & name, Robot * robot)
: ControlTask(name, robot)
{
  RCLCPP_INFO(get_logger(), "DwaController::DwaController");
}

DwaController::~DwaController()
{
  RCLCPP_INFO(get_logger(), "DwaController::~DwaController");
}

void
DwaController::execute()
{
  RCLCPP_INFO(get_logger(), "DwaController::execute");

  // Fake out some work
  for (int i = 0; i < 10; i++) {
    RCLCPP_INFO(get_logger(), "DwaController::execute: doing work: %d", i);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    // While we're doing the work, check if we've been preempted/canceled
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "DwaController::execute: task has been canceled");
      setCanceled();
      return;
    }
  }

  // TODO(mjeronimo): return the result
  RCLCPP_INFO(get_logger(), "DwaController::execute: task completed");
}
