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

#include <cmath>
#include <chrono>
#include <ctime>
#include <thread>
#include <memory>

#include "nav2_behaviors/back_up.hpp"

using nav2_tasks::TaskStatus;
using namespace std::chrono_literals;

namespace nav2_behaviors
{

BackUp::BackUp()
: Node("BackUp")
{
  RCLCPP_INFO(get_logger(), "Initializing the BackUp behavior");

  // TODO(orduno) Pull values from param server or robot
  max_linear_vel_ = 0.0;
  min_linear_vel_ = 0.0;
  linear_acc_lim_ = 0.0;
  goal_tolerance_distance_ = 0.0;

  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](auto) {});

  robot_ = std::make_unique<nav2_robot::Robot>(temp_node);

  vel_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  task_server_ = std::make_unique<nav2_tasks::BackUpTaskServer>(temp_node, false);
  task_server_->setExecuteCallback(std::bind(&BackUp::backUp, this, std::placeholders::_1));

  // Start listening for incoming BackUp task requests
  task_server_->startWorkerThread();

  RCLCPP_INFO(get_logger(), "Initialized the BackUp behavior");
}

BackUp::~BackUp()
{
  RCLCPP_INFO(get_logger(), "Shutting down the BackUp behavior");
}

nav2_tasks::TaskStatus BackUp::backUp(const nav2_tasks::BackUpCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "Attempting to back up");

  if (command->y != 0.0 || command->z != 0.0) {
    RCLCPP_INFO(get_logger(), "Backing up in Y and Z not supported, "
      "will only spin in Z.");
  }

  RCLCPP_INFO(get_logger(), "Currently only supported backing up by a fixed distance");
  TaskStatus status = timedBackup();

  nav2_tasks::BackUpResult result;
  task_server_->setResult(result);

  return status;
}

nav2_tasks::TaskStatus BackUp::timedBackup()
{
  auto start_time = std::chrono::system_clock::now();
  auto time_since_msg = std::chrono::system_clock::now();

  while (true) {
    if (task_server_->cancelRequested()) {
      RCLCPP_INFO(get_logger(), "Task cancelled");
      task_server_->setCanceled();
      return TaskStatus::CANCELED;
    }

    // TODO(orduno): For now, backing up is time based
    auto duration = 3s;

    // Log a message every second
    auto current_time = std::chrono::system_clock::now();
    if (current_time - time_since_msg >= 500ms) {
      RCLCPP_INFO(get_logger(), "Backing up...");
      time_since_msg = std::chrono::system_clock::now();
    }

    // Output control command
    geometry_msgs::msg::Twist cmd_vel;
    // TODO(orduno): assuming robot was moving fwd when it got stuck
    cmd_vel.linear.x = -0.05;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_->publish(cmd_vel);

    if (current_time - start_time >= duration) {
      cmd_vel.linear.x = 0.0;
      vel_pub_->publish(cmd_vel);
      RCLCPP_INFO(get_logger(), "Completed backing up");
      break;
    }
  }
  return TaskStatus::SUCCEEDED;
}

nav2_tasks::TaskStatus BackUp::controlledBackup()
{
  auto current_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

  if (!robot_->getCurrentPose(current_pose)) {
    RCLCPP_ERROR(get_logger(), "Current robot pose is not available.");
    return TaskStatus::FAILED;
  }

  // TODO(orduno): Implement controller for moving the robot by a given distance
  //               starting from the current pose

  RCLCPP_ERROR(get_logger(), "Back up controller not implement yet.");

  return TaskStatus::FAILED;
}

}  // namespace nav2_behaviors
