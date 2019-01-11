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
  // TODO(orduno) #378 Pull values from the robot
  max_linear_vel_ = 0.0;
  min_linear_vel_ = 0.0;
  linear_acc_lim_ = 0.0;

  default_vel_.linear.x = -0.05;
  default_vel_.linear.y = 0.0;
  default_vel_.angular.z = 0.0;

  default_exec_time_ = 5s;
  remaining_time_ = default_exec_time_;

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
  command_x_ = command->x;
  if (!robot_->getOdometry(initial_pose_)) {
    RCLCPP_ERROR(node_->get_logger(), "initial robot odom pose is not available.");
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

  if (distance >= abs(command_x_)) {
    cmd_vel.linear.x = 0;
    robot_->sendVelocity(cmd_vel);
    return TaskStatus::SUCCEEDED;
  }
  // TODO(mhpanah): cmd_vel value should be passed as a parameter
  command_x_ < 0 ? cmd_vel.linear.x = -0.025 : cmd_vel.linear.x = 0.025;
  robot_->sendVelocity(cmd_vel);

bool BackUp::pathIsClear()
{
  auto robot_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

  // TODO(orduno) Obtain the pose from the tf tree. The robot class is currently obtaining the pose
  //              from amcl, and amcl doesn't update the pose often enough.
  //              The robot pose might not be updated on sequential backup requests.
  if (!robot_->getCurrentPose(robot_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  nav2_world_model::FreeSpaceServiceRequest request;

  // Define the region size
  // Width is set to match the robot's diameter
  // Height is set to estimate the distance that will be traveled
  double robot_width = 0.22;  // TODO(orduno) get from robot class
  request.width = robot_width;
  request.height = default_vel_.linear.x * remaining_time_.count();

  // Define the reference point as the robot pose
  request.reference.x = robot_pose->pose.pose.position.x;
  request.reference.y = robot_pose->pose.pose.position.y;

  // Translate to set the edge of the region in front of the robot
  request.offset.x = 0.0;
  request.offset.y = robot_width / 2.0 + request.height / 2.0;

  // Rotate to match the opposite orientation of the robot's heading
  request.rotation = tf2::getYaw(robot_pose->pose.pose.orientation) + M_PI;

  RCLCPP_INFO(node_->get_logger(),
    "Checking if path is clear: w: %f, h: %f, rx: %f, ry: %f, rot: %f, ox: %f, oy: %f",
    request.width, request.height, request.reference.x, request.reference.y,
    request.rotation, request.offset.x, request.offset.y
  );

  return world_model_.confirmFreeSpace(request);
}

}  // namespace nav2_motion_primitives
