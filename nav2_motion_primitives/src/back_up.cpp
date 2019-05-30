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

using namespace std::chrono_literals;

namespace nav2_motion_primitives
{

BackUp::BackUp(rclcpp::Node::SharedPtr & node)
: MotionPrimitive<BackUpAction>(node, "BackUp")
{
  // TODO(orduno) #378 Pull values from the robot
  max_linear_vel_ = 0.0;
  min_linear_vel_ = 0.0;
  linear_acc_lim_ = 0.0;
}

BackUp::~BackUp()
{
}

Status BackUp::onRun(const std::shared_ptr<const BackUpAction::Goal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_INFO(node_->get_logger(), "Backing up in Y and Z not supported, "
      "will only move in X.");
  }

  command_x_ = command->target.x;

  if (!robot_->getOdometry(initial_pose_)) {
    RCLCPP_ERROR(node_->get_logger(), "initial robot odom pose is not available.");
    return Status::FAILED;
  }

  return Status::SUCCEEDED;
}

Status BackUp::onCycleUpdate()
{
  auto current_odom_pose = std::shared_ptr<nav_msgs::msg::Odometry>();

  if (!robot_->getOdometry(current_odom_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot odom is not available.");
    return Status::FAILED;
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  double diff_x = initial_pose_->pose.pose.position.x - current_odom_pose->pose.pose.position.x;
  double diff_y = initial_pose_->pose.pose.position.y - current_odom_pose->pose.pose.position.y;
  double distance = sqrt(diff_x * diff_x + diff_y * diff_y);

  if (distance >= abs(command_x_)) {
    cmd_vel.linear.x = 0;
    robot_->sendVelocity(cmd_vel);
    return Status::SUCCEEDED;
  }
  // TODO(mhpanah): cmd_vel value should be passed as a parameter
  command_x_ < 0 ? cmd_vel.linear.x = -0.025 : cmd_vel.linear.x = 0.025;
  robot_->sendVelocity(cmd_vel);

  return Status::RUNNING;
}

}  // namespace nav2_motion_primitives
