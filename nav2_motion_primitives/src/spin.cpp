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
#include "nav2_util/angleutils.hpp"

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
  max_rotational_vel_ = 0.1;
  min_rotational_vel_ = 0.01;
  rotational_acc_lim_ = 3.2;
  spin_command_ = 0.0;
  start_yaw_ = 0.0;
  prev_yaw_ = 0.0;
  origin_cross_count_ = 0;
}

Spin::~Spin()
{
}

TaskStatus Spin::onRun(const SpinCommand::SharedPtr command)
{
  // Get the amount to spin
  double pitch, roll;
  tf2::getEulerYPR(command->quaternion, spin_command_, pitch, roll);

  RCLCPP_INFO(node_->get_logger(),
    "Spin request is yaw: %.2f deg, pitch: %.2f deg, roll: %.2f deg",
    spin_command_ * 180.0 / M_PI,
    pitch * 180.0 / M_PI,
    roll * 180.0 / M_PI);

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

  origin_cross_count_ = 0;
  prev_yaw_ = start_yaw_;

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

  // TODO(orduno) Currently the yaw is not updated frequently enough
  //              therefore, sometimes we overshoot the target angle
  if (!getYaw(current_yaw)) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot yaw is not available.");
    return TaskStatus::FAILED;
  }

  // Update the count of origin crossings to handle discontinuity at 180,-180.
  updateOriginCrossing(current_yaw);

  double traveled_angle = angleutils::normalize(current_yaw) - angleutils::normalize(start_yaw_);

  // Adjust for origin discontinuity
  traveled_angle = traveled_angle + 2.0 * origin_cross_count_ * M_PI;

  double dist_left = angleutils::normalize(spin_command_) - traveled_angle;

  RCLCPP_DEBUG(node_->get_logger(),
    "Yaws [deg], starting: %.2f current: %.2f, traveled: %.2f, remaining: %.2f",
    start_yaw_ * 180.0 / M_PI, current_yaw * 180.0 / M_PI,
    traveled_angle * 180.0 / M_PI, dist_left * 180.0 / M_PI);

  // Log a message every second
  auto current_time = std::chrono::system_clock::now();
  if (current_time - time_since_msg >= 1s) {
    RCLCPP_INFO(node_->get_logger(),
      "Traveled %.2f deg with %.2f deg left",
      traveled_angle * 180.0 / M_PI, dist_left * 180.0 / M_PI);
    time_since_msg = std::chrono::system_clock::now();
  }

  // TODO(orduno) #379 forward simulation to check if future position is feasible

  // compute the velocity that will let us stop by the time we reach the goal
  // v_f^2 == v_i^2 + 2 * a * d
  // solving for v_i if v_f = 0
  double vel = sqrt(2 * rotational_acc_lim_ * std::abs(dist_left));

  // limit velocity
  int vel_dir = (spin_command_ > 0) ? 1.0 : -1.0;
  vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_) * vel_dir;

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = vel;

  robot_->sendVelocity(cmd_vel);

  // check if we are done
  if (
    ((spin_command_ > 0) && (traveled_angle >= angleutils::normalize(spin_command_))) ||
    ((spin_command_ < 0) && (traveled_angle <= angleutils::normalize(spin_command_))))
  {
    RCLCPP_INFO(node_->get_logger(),
      "Robot spinned %.2f", traveled_angle * 180.0 / M_PI);
    return TaskStatus::SUCCEEDED;
  }

  return TaskStatus::RUNNING;
}

void Spin::updateOriginCrossing(double current_yaw)
{
  // TODO(orduno) Assuming measurement reading are not too noisy
  //              otherwise we might have multiple counts
  if (current_yaw - prev_yaw_ < -M_PI) {
    // crossed while rotating CCW
    origin_cross_count_++;
  } else if (prev_yaw_ - current_yaw < -M_PI) {
    // crossed while rotating CW
    origin_cross_count_--;
  }
  prev_yaw_ = current_yaw;
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
