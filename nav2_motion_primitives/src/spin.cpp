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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;

namespace nav2_motion_primitives
{

Spin::Spin(rclcpp::Node::SharedPtr & node)
: MotionPrimitive<SpinAction>(node, "Spin")
{
  // TODO(orduno) #378 Pull values from the robot
  max_rotational_vel_ = 1.0;
  min_rotational_vel_ = 0.4;
  rotational_acc_lim_ = 3.2;
  goal_tolerance_angle_ = 0.10;
  start_yaw_ = 0.0;
}

Spin::~Spin()
{
}

Status Spin::onRun(const std::shared_ptr<const SpinAction::Goal> command)
{
  double yaw, pitch, roll;
  tf2::getEulerYPR(command->target.quaternion, yaw, pitch, roll);

  if (roll != 0.0 || pitch != 0.0) {
    RCLCPP_INFO(node_->get_logger(), "Spinning on Y and X not supported, "
      "will only spin in Z.");
  }

  RCLCPP_INFO(node_->get_logger(), "Currently only supported spinning by a fixed amount");

  start_time_ = std::chrono::system_clock::now();

  return Status::SUCCEEDED;
}

Status Spin::onCycleUpdate()
{
  // Currently only an open-loop time-based controller is implemented
  // The closed-loop version 'controlledSpin()' has not been fully tested
  return timedSpin();
}

Status Spin::timedSpin()
{
  // Output control command
  geometry_msgs::msg::Twist cmd_vel;

  // TODO(orduno) #423 fixed speed
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.5;
  robot_->sendVelocity(cmd_vel);

  // TODO(orduno) #423 fixed time
  auto current_time = std::chrono::system_clock::now();
  if (current_time - start_time_ >= 6s) {  // almost 180 degrees
    // Stop the robot
    cmd_vel.angular.z = 0.0;
    robot_->sendVelocity(cmd_vel);

    return Status::SUCCEEDED;
  }

  return Status::RUNNING;
}

Status Spin::controlledSpin()
{
  // TODO(orduno) #423 Test and tune controller
  //              check it doesn't abruptly start and stop
  //              or cause massive wheel slippage when accelerating

  // Get current robot orientation
  auto current_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  if (!robot_->getCurrentPose(current_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
    return Status::FAILED;
  }

  double current_yaw = tf2::getYaw(current_pose->pose.pose.orientation);

  double current_angle = current_yaw - start_yaw_;

  double dist_left = M_PI - current_angle;

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

  // check if we are done
  if (dist_left >= (0.0 - goal_tolerance_angle_)) {
    return Status::SUCCEEDED;
  }

  return Status::RUNNING;
}

}  // namespace nav2_motion_primitives
