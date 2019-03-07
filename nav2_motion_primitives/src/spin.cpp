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
#include <algorithm>
#include <memory>

#include "nav2_motion_primitives/spin.hpp"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using nav2_tasks::TaskStatus;
using nav2_tasks::SpinCommand;
using nav2_tasks::SpinResult;
using namespace std::chrono_literals;

namespace nav2_motion_primitives
{

Spin::Spin(rclcpp::Node::SharedPtr & node)
: MotionPrimitive<nav2_tasks::SpinCommand, nav2_tasks::SpinResult>(node)
{
  // TODO(orduno) #378 Pull values from the robot
  max_rotational_vel_ = 0.5;
  min_rotational_vel_ = 0.1;
  rotational_acc_lim_ = 3.2;
  spin_command_ = 0.0;
  goal_tolerance_angle_ = 0.10;
  start_yaw_ = 0.0;
}

Spin::~Spin()
{
}

TaskStatus Spin::onRun(const SpinCommand::SharedPtr command)
{
  // Get the amount to spin
  double pitch, roll;
  tf2::getEulerYPR(command->quaternion, spin_command_, pitch, roll);

  if (roll != 0.0 || pitch != 0.0) {
    RCLCPP_INFO(node_->get_logger(), "Spinning on Y and X not supported, "
      "will only spin in Z.");
  }

  // Get the current starting yaw
  if (!getYaw(start_yaw_)) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot yaw is not available.");
    return TaskStatus::FAILED;
  }

  time_since_msg = std::chrono::system_clock::now();

  return TaskStatus::SUCCEEDED;
}

TaskStatus Spin::onCycleUpdate(SpinResult & result)
{
  TaskStatus status = controlledSpin();

  // For now sending an empty task result
  nav2_tasks::SpinResult empty_result;
  result = empty_result;

  return status;
}

TaskStatus Spin::controlledSpin()
{
  // Get current robot orientation
  double current_yaw;

  if (!getYaw(current_yaw)) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot yaw is not available.");
    return TaskStatus::FAILED;
  }

  double current_angle = current_yaw - start_yaw_;
  double dist_left = spin_command_ - current_angle;

  // TODO(orduno) #379 forward simulation to check if future position is feasible

  // compute the velocity that will let us stop by the time we reach the goal
  // v_f^2 == v_i^2 + 2 * a * d
  // solving for v_i if v_f = 0
  double vel = sqrt(2 * rotational_acc_lim_ * dist_left);

  // limit velocity
  vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = vel;

  robot_->sendVelocity(cmd_vel);

  // Log a message every second
  auto current_time = std::chrono::system_clock::now();
  if (current_time - time_since_msg >= 1s) {
    RCLCPP_INFO(node_->get_logger(),
      "Spin::ControlledSpin, got more %.2f degrees to move", dist_left * 180.0 / M_PI);
    time_since_msg = std::chrono::system_clock::now();
  }

  // check if we are done
  if (dist_left >= (0.0 - goal_tolerance_angle_)) {
    return TaskStatus::SUCCEEDED;
  }

  return TaskStatus::RUNNING;
}

bool Spin::getYaw(double & yaw) const
{
  auto current_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  if (!robot_->getCurrentPose(current_pose)) {
    return false;
  }

  yaw = tf2::getYaw(current_pose->pose.pose.orientation);
  return true;
}

}  // namespace nav2_motion_primitives
