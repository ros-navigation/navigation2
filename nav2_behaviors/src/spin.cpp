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

#include "nav2_behaviors/spin.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using nav2_tasks::TaskStatus;
using namespace std::chrono_literals;

namespace nav2_behaviors
{

Spin::Spin() : Node("Spin")
{
  RCLCPP_INFO(get_logger(), "Initializing the Spin behavior");

  // TODO(orduno) Pull values from param server or robot
  max_rotational_vel_ = 1.0;
  min_rotational_vel_ = 0.4;
  rotational_acc_lim_ = 3.2;
  goal_tolerance_angle_ = 0.10;

  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](auto) {});

  robot_ = std::make_unique<nav2_robot::Robot>(temp_node);

  vel_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  task_server_ = std::make_unique<nav2_tasks::SpinTaskServer>(temp_node, false);
  task_server_->setExecuteCallback(std::bind(&Spin::spin, this, std::placeholders::_1));

  // Start listening for incoming Spin task requests
  task_server_->startWorkerThread();

  RCLCPP_INFO(get_logger(), "Initialized the Spin  behavior");
}

Spin::~Spin()
{
  RCLCPP_INFO(get_logger(), "Shutting down the Spin behavior");
}

nav2_tasks::TaskStatus Spin::spin(const nav2_tasks::SpinCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "Attempting to spin");

  double yaw, pitch, roll;
  getAnglesFromQuaternion(command->quaternion, yaw, pitch, roll);

  if (roll != 0.0 || pitch != 0.0) {
    RCLCPP_INFO(get_logger(), "Spinning on Y and X not supported currently"
    " , will only spin in Z.");
  } else if ((yaw * 180.0 / M_PI) != 180.0) {
    RCLCPP_INFO(get_logger(), "Spinning by 180deg is only supported");
  }

  // Before spinning we need to back up a bit
  // TODO(orduno): Create a separate behavior and bt node for backing up
  time_based_backup();

  time_based_spin();

  nav2_tasks::SpinResult result;
  task_server_->setResult(result);

  return TaskStatus::SUCCEEDED;
}

nav2_tasks::TaskStatus Spin::time_based_backup()
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
    // TODO (orduno): assuming robot was moving fwd when it got stuck
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

nav2_tasks::TaskStatus Spin::time_based_spin()
{
  auto start_time = std::chrono::system_clock::now();
  auto time_since_msg = std::chrono::system_clock::now();

  while (true) {
    if (task_server_->cancelRequested()) {
      RCLCPP_INFO(get_logger(), "Task cancelled");
      task_server_->setCanceled();
      return TaskStatus::CANCELED;
    }

    // TODO(orduno): For now, spinning is time based
    auto duration = 4s;

    // Log a message every second
    auto current_time = std::chrono::system_clock::now();
    if (current_time - time_since_msg >= 500ms) {
      RCLCPP_INFO(get_logger(), "Spinning...");
      time_since_msg = std::chrono::system_clock::now();
    }

    // Output control command
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.5;
    vel_pub_->publish(cmd_vel);

    if (current_time - start_time >= duration) {
      cmd_vel.angular.z = 0.0;
      vel_pub_->publish(cmd_vel);
      RCLCPP_INFO(get_logger(), "Completed rotation");
      break;
    }
  }
  return TaskStatus::SUCCEEDED;
}

nav2_tasks::TaskStatus Spin::controller_based_spin()
{
  // Get current robot orientation
  double start_yaw, current_yaw;

  if (!getRobotYaw(start_yaw)) {
    return TaskStatus::FAILED;
  }

  while (true) {
    if (task_server_->cancelRequested()) {
      RCLCPP_INFO(get_logger(), "Task cancelled");
      task_server_->setCanceled();
      return TaskStatus::CANCELED;
    }

    getRobotYaw(current_yaw);

    double current_angle = current_yaw - start_yaw;

    double dist_left = M_PI - current_angle;

    // TODO(orduno) forward simulation to check if future position is feasible

    // compute the velocity that will let us stop by the time we reach the goal
    // v_f^2 == v_i^2 + 2 * a * d
    // solving for v_i if v_f = 0
    double vel = sqrt(2 * rotational_acc_lim_ * dist_left);

    // limit velocity
    vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = vel;

    vel_pub.publish(cmd_vel);

    // check if we are done
    if (dist_left >= (0.0 - goal_tolerance_angle_)) {
      break;
    }
  }

  return TaskStatus::SUCCEEDED;
}

void Spin::getAnglesFromQuaternion(
  const geometry_msgs::msg::Quaternion & quaternion,
  double & yaw, double & pitch, double & roll)
{
  tf2::Matrix3x3(
  tf2::Quaternion(
    command->quaternion.x, command->quaternion.y,
    command->quaternion.z, command->quaternion.w)).getEulerYPR(yaw, pitch, roll);
}

bool Spin::getRobotYaw(double & yaw)
{
  auto current_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

  if (!robot_->getCurrentPose(current_pose)) {
    RCLCPP_ERROR(get_logger(), "Current robot pose is not available.");
    return false
  }

  double pitch, roll;
  getAnglesFromQuaternion(current_pose->pose.orientation, yaw, pitch, roll);

  return true;
}

}  // namespace nav2_behaviors
