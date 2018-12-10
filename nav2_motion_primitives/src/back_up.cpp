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

#include <chrono>
#include <ctime>
#include <memory>

#include "nav2_motion_primitives/back_up.hpp"

using nav2_tasks::TaskStatus;
using namespace std::chrono_literals;

namespace nav2_motion_primitives
{

BackUp::BackUp(rclcpp::Node::SharedPtr & node)
: MotionPrimitive<nav2_tasks::BackUpCommand, nav2_tasks::BackUpResult>(node)
{
  // TODO(orduno) #378 Pull values from the robot
  max_linear_vel_ = 0.0;
  min_linear_vel_ = 0.0;
  linear_acc_lim_ = 0.0;

  start_time_ = std::chrono::system_clock::now();
}

BackUp::~BackUp()
{
}

nav2_tasks::TaskStatus BackUp::onRun(const nav2_tasks::BackUpCommand::SharedPtr command)
{
  if (command->y != 0.0 || command->z != 0.0) {
    RCLCPP_INFO(node_->get_logger(), "Backing up in Y and Z not supported, "
      "will only move in X.");
  }

  RCLCPP_INFO(node_->get_logger(), "Currently only supported backing up by a fixed distance");

  start_time_ = std::chrono::system_clock::now();

  return nav2_tasks::TaskStatus::SUCCEEDED;
}

nav2_tasks::TaskStatus BackUp::onCycleUpdate(nav2_tasks::BackUpResult & result)
{
  // Currently only an open-loop controller is implemented
  // TODO(orduno) #423 Create a base class for open-loop controlled motion_primitives
  TaskStatus status = timedBackup();

  // For now sending an empty task result
  nav2_tasks::BackUpResult empty_result;
  result = empty_result;

  return status;
}

nav2_tasks::TaskStatus BackUp::timedBackup()
{
  // Output control command
  geometry_msgs::msg::Twist cmd_vel;

  // TODO(orduno): #423 assuming robot was moving fwd when it got stuck
  //               fixed speed
  cmd_vel.linear.x = -0.025;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  robot_->sendVelocity(cmd_vel);

// TODO(orduno): #423 fixed time
  auto current_time = std::chrono::system_clock::now();
  if (current_time - start_time_ >= 3s) {
    // Stop the robot
    cmd_vel.linear.x = 0.0;
    robot_->sendVelocity(cmd_vel);

    return TaskStatus::SUCCEEDED;
  }

  return TaskStatus::RUNNING;
}

nav2_tasks::TaskStatus BackUp::controlledBackup()
{
  auto current_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

  if (!robot_->getCurrentPose(current_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
    return TaskStatus::FAILED;
  }

  // TODO(orduno): #423 Implement controller for moving the robot by a given distance
  //               starting from the current pose

  RCLCPP_ERROR(node_->get_logger(), "Back up controller not implement yet.");

  return TaskStatus::FAILED;
}

}  // namespace nav2_motion_primitives
