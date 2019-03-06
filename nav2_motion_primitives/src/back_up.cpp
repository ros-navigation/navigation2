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
#include <cmath>

#include "nav2_motion_primitives/back_up.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using nav2_tasks::TaskStatus;
using namespace std::chrono_literals;

namespace nav2_motion_primitives
{

BackUp::BackUp(rclcpp::Node::SharedPtr & node)
: MotionPrimitive<nav2_tasks::BackUpCommand, nav2_tasks::BackUpResult>(node)
{
  node_->get_parameter_or<double>("backup_velocity", default_vel_.linear.x, 0.025);
  default_vel_.linear.y = 0.0;
  default_vel_.angular.z = 0.0;
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

  command_x_ = command->x;

  if (!robot_->getOdometry(initial_pose_)) {
    RCLCPP_ERROR(node_->get_logger(), "Initial robot odom pose is not available.");
    return nav2_tasks::TaskStatus::FAILED;
  }

  return nav2_tasks::TaskStatus::SUCCEEDED;
}

nav2_tasks::TaskStatus BackUp::onCycleUpdate(nav2_tasks::BackUpResult & result)
{
  TaskStatus status = controlledBackup();

  // For now sending an empty task result
  nav2_tasks::BackUpResult empty_result;
  result = empty_result;

  return status;
}

nav2_tasks::TaskStatus BackUp::controlledBackup()
{
  auto current_odom_pose = std::shared_ptr<nav_msgs::msg::Odometry>();

  if (!robot_->getOdometry(current_odom_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot odom is not available.");
    return TaskStatus::FAILED;
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  double diff_x = initial_pose_->pose.pose.position.x - current_odom_pose->pose.pose.position.x;
  double diff_y = initial_pose_->pose.pose.position.y - current_odom_pose->pose.pose.position.y;
  double distance = sqrt(diff_x * diff_x + diff_y * diff_y);

  if (distance >= std::abs(command_x_)) {
    cmd_vel.linear.x = 0;
    robot_->sendVelocity(cmd_vel);
    RCLCPP_INFO(node_->get_logger(), "Completed backup, traveled %0.2f", distance);
    return TaskStatus::SUCCEEDED;
  }

  // TODO(mhpanah): cmd_vel value should be passed as a parameter
  if (command_x_ < 0) {
    cmd_vel.linear.x = -1.0 * default_vel_.linear.x;
  } else {
    cmd_vel.linear.x = default_vel_.linear.x;
  }

  robot_->sendVelocity(cmd_vel);

  return TaskStatus::RUNNING;
}

bool BackUp::pathIsClear()
{
  auto robot_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

  nav2_world_model::FreeSpaceServiceRequest request;

  // Check for clear path relative to robot.
  request.frame_id = "base_link";

  double robot_width = robot_->getRadius();
  double direction = (command_x_ < 0.0) ? -1.0 : 1.0;

  // Define the two opposite corners of the region to check.
  // As coordinates are provided on robot's frame (base_link),
  // the X axis is positive in the direction the robot if facing
  // and Y axis is positive to the left.

  // Corner closest to robot
  request.region.corner.x = direction * robot_width / 2.0;
  request.region.corner.y = robot_width / 2.0;
  request.region.corner.z = 0.0;

  // Corner furthest from robot
  request.region.opposite_corner.x = direction * robot_width / 2.0 + command_x_;
  request.region.opposite_corner.y = -1.0 * robot_width / 2.0;
  request.region.opposite_corner.z = 0.0;

  return world_model_.confirmFreeSpace(request);
}

}  // namespace nav2_motion_primitives
