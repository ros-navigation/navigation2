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
  goal_tolerance_angle_ = 0.10;
  start_yaw_ = 0.0;
}

Spin::~Spin()
{
}

nav2_tasks::TaskStatus Spin::onRun(const nav2_tasks::SpinCommand::SharedPtr command)
{
  double pitch, roll;
  tf2::getEulerYPR(command->quaternion, start_yaw_, pitch, roll);
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
  const double current_yaw = tf2::getYaw(current_pose->pose.pose.orientation);
  const double current_angle_dist = current_yaw - start_yaw_;
  const double dist_left = M_PI - current_angle_dist;

  // TODO(stevemacenski) #553 check for collisions

  // Compute the velocity, clip, and send
  double vel = sqrt(2 * rotational_acc_lim_ * dist_left);
  vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = vel;

  robot_->sendVelocity(cmd_vel);

  // Termination conditions
  if (fabs(dist_left) < goal_tolerance_angle_) {
    return TaskStatus::SUCCEEDED;
  }

  return TaskStatus::RUNNING;
}

}  // namespace nav2_motion_primitives
