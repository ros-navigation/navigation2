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

#include "nav2_recoveries/back_up.hpp"

using namespace std::chrono_literals;

namespace nav2_recoveries
{

BackUp::BackUp(rclcpp::Node::SharedPtr & node)
: Recovery<BackUpAction>(node, "BackUp")
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

  if (!getRobotPose(initial_pose_)) {
    RCLCPP_ERROR(node_->get_logger(), "Initial robot pose is not available.");
    return Status::FAILED;
  }

  return Status::SUCCEEDED;
}

Status BackUp::onCycleUpdate()
{
  geometry_msgs::msg::Pose current_pose;
  if (!getRobotPose(current_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
    return Status::FAILED;
  }

  double diff_x = initial_pose_.position.x - current_pose.position.x;
  double diff_y = initial_pose_.position.y - current_pose.position.y;
  double distance = sqrt(diff_x * diff_x + diff_y * diff_y);

  if (distance >= abs(command_x_)) {
    stopRobot();
    return Status::SUCCEEDED;
  }
  // TODO(mhpanah): cmd_vel value should be passed as a parameter
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  command_x_ < 0 ? cmd_vel.linear.x = -0.025 : cmd_vel.linear.x = 0.025;

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.position.x + cmd_vel.linear.x * (1 / cycle_frequency_);
  pose2d.y = current_pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.orientation);

  if (!collision_checker_->isCollisionFree(pose2d)) {
    stopRobot();
    RCLCPP_WARN(node_->get_logger(), "Collision Ahead - Exiting BackUp");
    return Status::SUCCEEDED;
  }

  vel_publisher_->publishCommand(cmd_vel);

  return Status::RUNNING;
}

}  // namespace nav2_recoveries
