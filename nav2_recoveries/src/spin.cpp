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

#include "nav2_recoveries/spin.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;

namespace nav2_recoveries
{

Spin::Spin(rclcpp::Node::SharedPtr & node, std::shared_ptr<tf2_ros::Buffer> tf)
: Recovery<SpinAction>(node, "Spin", tf)
{
  // TODO(orduno) #378 Pull values from the robot
  max_rotational_vel_ = 1.0;
  min_rotational_vel_ = 0.4;
  rotational_acc_lim_ = 3.2;
  prev_yaw_ = 0.0;
  delta_yaw_ = 0.0;
  relative_yaw_ = 0.0;
}

Spin::~Spin()
{
}

Status Spin::onRun(const std::shared_ptr<const SpinAction::Goal> command)
{
  cmd_yaw_ = -command->target_yaw;
  return Status::SUCCEEDED;
}

Status Spin::onCycleUpdate()
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, tf_)) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
    return Status::FAILED;
  }

  const double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  delta_yaw_ = abs(abs(current_yaw) - abs(prev_yaw_));
  relative_yaw_ += delta_yaw_;
  const double yaw_diff = relative_yaw_ - cmd_yaw_;

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  if (relative_yaw_ >= abs(cmd_yaw_)) {
    relative_yaw_ = 0.0;
    stopRobot();
    return Status::SUCCEEDED;
  }


  double vel = sqrt(2 * rotational_acc_lim_ * abs(yaw_diff));
  vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

  cmd_yaw_ < 0 ? cmd_vel.angular.z = -vel : cmd_vel.angular.z = vel;

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation) +
    cmd_vel.angular.z * (1 / cycle_frequency_);

  if (!collision_checker_->isCollisionFree(pose2d)) {
    stopRobot();
    RCLCPP_WARN(node_->get_logger(), "Collision Ahead - Exiting Spin ");
    return Status::SUCCEEDED;
  }

  prev_yaw_ = current_yaw;
  vel_pub_->publish(cmd_vel);

  return Status::RUNNING;
}

}  // namespace nav2_recoveries
