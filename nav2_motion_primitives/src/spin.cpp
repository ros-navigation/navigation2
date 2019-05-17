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

using nav2_tasks::TaskStatus;
using namespace std::chrono_literals;

namespace nav2_motion_primitives
{

Spin::Spin(rclcpp::Node::SharedPtr & node)
: MotionPrimitive<nav2_tasks::SpinCommand, nav2_tasks::SpinResult>(node)
{
  // TODO(orduno) #378 Pull values from the robot
  max_rotational_vel_ = 1.0;
  min_rotational_vel_ = 0.4;
  rotational_acc_lim_ = 3.2;
  goal_tolerance_angle_ = 0.17;
  direction_ = 1.0;
  commanded_dist_ = 0.0;
}

Spin::~Spin()
{
}

nav2_tasks::TaskStatus Spin::onRun(const nav2_tasks::SpinCommand::SharedPtr command)
{
  double pitch_holder, roll_holder;
  if (!robot_->getOdometry(initial_pose_)) {
    RCLCPP_ERROR(node_->get_logger(), "initial robot odom pose is not available.");
    return nav2_tasks::TaskStatus::FAILED;
  }

  // getEulerYPR are normalized angles (-M_PI, M_PI]
  tf2::getEulerYPR(command->quaternion, commanded_dist_, pitch_holder, roll_holder);

  if (roll_holder != 0.0 || pitch_holder != 0.0) {
    RCLCPP_WARN(node_->get_logger(), "Spinning on Y and X not supported, "
      "will only spin in Z.");
  }

  // gets unsigned distance and turning direction
  if (commanded_dist_ < 0.0) {
    commanded_dist_ += 2 * M_PI;
    direction_ = -1.0;
  } else {
    direction_ = 1.0;
  }

  return nav2_tasks::TaskStatus::SUCCEEDED;
}

nav2_tasks::TaskStatus Spin::onCycleUpdate(nav2_tasks::SpinResult & result)
{
  TaskStatus status = rotateSome();
  nav2_tasks::SpinResult empty_result;
  result = empty_result;
  return status;
}

nav2_tasks::TaskStatus Spin::rotateSome()
{
  // Get current robot orientation
  auto current_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  if (!robot_->getCurrentPose(current_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
    return TaskStatus::FAILED;
  }

  // Get distance left
  const double current_angle_dist = tf2::angleShortestPath(current_pose, initial_pose_);
  const double dist_left = commanded_dist_ - fabs(current_angle_dist);

  // TODO(stevemacenski) #533 check for collisions

  // Compute the velocity, clip, and send
  double vel = sqrt(2 * rotational_acc_lim_ * dist_left);
  vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = vel * direction_;

  robot_->sendVelocity(cmd_vel);

  // Termination conditions
  if (dist_left < goal_tolerance_angle_) {
    cmd_vel.angular.z = 0.0;
    robot_->sendVelocity(cmd_vel);
    return TaskStatus::SUCCEEDED;
  }

  return TaskStatus::RUNNING;
}

}  // namespace nav2_motion_primitives
