// Copyright (c) 2018 Intel Corporation, 2019 Samsung Research America
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
#include <thread>
#include <algorithm>
#include <memory>
#include <utility>

#include "nav2_behaviors/plugins/spin.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;

namespace nav2_behaviors
{

Spin::Spin()
: TimedBehavior<SpinAction>(),
  feedback_(std::make_shared<SpinAction::Feedback>()),
  min_rotational_vel_(0.0),
  max_rotational_vel_(0.0),
  rotational_acc_lim_(0.0),
  cmd_yaw_(0.0),
  prev_yaw_(0.0),
  relative_yaw_(0.0),
  simulate_ahead_time_(0.0)
{
}

Spin::~Spin() = default;

void Spin::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node,
    "simulate_ahead_time", rclcpp::ParameterValue(2.0));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "max_rotational_vel", rclcpp::ParameterValue(1.0));
  node->get_parameter("max_rotational_vel", max_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "min_rotational_vel", rclcpp::ParameterValue(0.4));
  node->get_parameter("min_rotational_vel", min_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "rotational_acc_lim", rclcpp::ParameterValue(3.2));
  node->get_parameter("rotational_acc_lim", rotational_acc_lim_);
}

ResultStatus Spin::onRun(const std::shared_ptr<const SpinActionGoal> command)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, local_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return ResultStatus{Status::FAILED, SpinActionResult::TF_ERROR};
  }

  prev_yaw_ = tf2::getYaw(current_pose.pose.orientation);
  relative_yaw_ = 0.0;

  cmd_yaw_ = command->target_yaw;
  RCLCPP_INFO(
    logger_, "Turning %0.2f for spin behavior.",
    cmd_yaw_);

  command_time_allowance_ = command->time_allowance;
  end_time_ = this->clock_->now() + command_time_allowance_;

  return ResultStatus{Status::SUCCEEDED, SpinActionResult::NONE};
}

ResultStatus Spin::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "Exceeded time allowance before reaching the Spin goal - Exiting Spin");
    return ResultStatus{Status::FAILED, SpinActionResult::TIMEOUT};
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, local_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return ResultStatus{Status::FAILED, SpinActionResult::TF_ERROR};
  }

  const double current_yaw = tf2::getYaw(current_pose.pose.orientation);

  double delta_yaw = current_yaw - prev_yaw_;
  if (abs(delta_yaw) > M_PI) {
    delta_yaw = copysign(2 * M_PI - abs(delta_yaw), prev_yaw_);
  }

  relative_yaw_ += delta_yaw;
  prev_yaw_ = current_yaw;

  feedback_->angular_distance_traveled = static_cast<float>(relative_yaw_);
  action_server_->publish_feedback(feedback_);

  double remaining_yaw = abs(cmd_yaw_) - abs(relative_yaw_);
  if (remaining_yaw < 1e-6) {
    stopRobot();
    return ResultStatus{Status::SUCCEEDED, SpinActionResult::NONE};
  }

  double vel = sqrt(2 * rotational_acc_lim_ * remaining_yaw);
  vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.frame_id = robot_base_frame_;
  cmd_vel->header.stamp = clock_->now();
  cmd_vel->twist.angular.z = copysign(vel, cmd_yaw_);

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(relative_yaw_, cmd_vel->twist, pose2d)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead - Exiting Spin");
    return ResultStatus{Status::FAILED, SpinActionResult::COLLISION_AHEAD};
  }

  vel_pub_->publish(std::move(cmd_vel));

  return ResultStatus{Status::RUNNING, SpinActionResult::NONE};
}

bool Spin::isCollisionFree(
  const double & relative_yaw,
  const geometry_msgs::msg::Twist & cmd_vel,
  geometry_msgs::msg::Pose2D & pose2d)
{
  // Simulate ahead by simulate_ahead_time_ in cycle_frequency_ increments
  int cycle_count = 0;
  double sim_position_change;
  const int max_cycle_count = static_cast<int>(cycle_frequency_ * simulate_ahead_time_);
  geometry_msgs::msg::Pose2D init_pose = pose2d;
  bool fetch_data = true;

  while (cycle_count < max_cycle_count) {
    sim_position_change = cmd_vel.angular.z * (cycle_count / cycle_frequency_);
    pose2d.theta = init_pose.theta + sim_position_change;
    cycle_count++;

    if (abs(relative_yaw) - abs(sim_position_change) <= 0.) {
      break;
    }

    if (!local_collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }
  return true;
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Spin, nav2_core::Behavior)
