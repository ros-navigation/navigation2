// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include "nav2_controller_example/dwa_controller.hpp"

using namespace std::chrono_literals;
using nav2_tasks::TaskStatus;

namespace nav2_controller_example
{

DwaController::DwaController()
: nav2_tasks::FollowPathTaskServer("FollowPathNode")
{
  RCLCPP_INFO(get_logger(), "Initializing DwaController");
  vel_pub_ = this->create_publisher<CmdVel>("/mobile_base/commands/velocity", 1);
}

DwaController::~DwaController()
{
  RCLCPP_INFO(get_logger(), "Shutting down DwaController");
}

TaskStatus
DwaController::execute(const nav2_tasks::FollowPathCommand::SharedPtr command)
{
  if (command->poses.size() == 0) {
    RCLCPP_WARN(get_logger(), "Received an empty path");
  } else {
    RCLCPP_INFO(get_logger(), "Received a new path to follow of"
      " size %i ending at (%.2f, %.2f)", (int)command->poses.size(),
      command->poses.back().position.x,
      command->poses.back().position.y);
  }

  unsigned int seed = std::time(nullptr);

  // Let's use a random number of iterations (from 10 to 30)
  int iterations = (rand_r(&seed) % 20) + 10;

  // Spin here for a bit to fake out some processing time
  for (int i = 0; i < iterations; i++) {
    // Do a bit of the task
    const double cmd_vel = 0.1;
    sendVelocity(cmd_vel);
    std::this_thread::sleep_for(250ms);

    // Before we loop again to do more work, check if we've been canceled
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "Follow task has been canceled");
      setCanceled();
      return TaskStatus::CANCELED;
    }
  }

  // Stop the robot after having it move forward for a bit
  sendVelocity(0);

  // We've successfully completed the task, so return the result
  RCLCPP_INFO(get_logger(), "Follow task has been completed");

  nav2_tasks::FollowPathResult result;
  setResult(result);

  return TaskStatus::SUCCEEDED;
}

void DwaController::sendVelocity(double speed)
{
  RCLCPP_INFO(get_logger(), "Send velocity: %f", speed);

  CmdVel v;
  v.linear.x = speed;
  v.linear.y = 0;
  v.linear.z = 0;
  v.angular.x = 0;
  v.angular.y = 0;
  v.angular.z = 0;
  vel_pub_->publish(v);
}

}  // namespace nav2_controller_example
