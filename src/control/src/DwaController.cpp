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
#include "control/DwaController.hpp"
#include <chrono>

using namespace std::chrono_literals;

DwaController::DwaController(const std::string & name)
: FollowPathTaskServer(name)
{
  RCLCPP_INFO(get_logger(), "DwaController::DwaController");
}

DwaController::~DwaController()
{
  RCLCPP_INFO(get_logger(), "DwaController::~DwaController");
}

TaskStatus
DwaController::executeAsync(const FollowPathCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "DwaController::executeAsync");

  // Spin here for a bit to fake out some processing time
  for (int i = 0; i < 10; i++) {
    // Do a bit of the task
    RCLCPP_INFO(get_logger(), "DwaController::executeAsync: doing work: %d", i);
    std::this_thread::sleep_for(250ms);

    // Before we loop again to do more work, check if we've been canceled
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "DwaController::executeAsync: task has been canceled");
      setCanceled();
      return TaskStatus::CANCELED;
    }
  }

  // We've successfully completed the task, so return the result
  RCLCPP_INFO(get_logger(), "DwaController::executeAsync: task completed");

  FollowPathResult result;
  setResult(result);

  return TaskStatus::SUCCEEDED;
}

