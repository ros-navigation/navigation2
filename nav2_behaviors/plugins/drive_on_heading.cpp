// Copyright (c) 2022 Joshua Wallace
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
#include <utility>

#include "drive_on_heading.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_msgs/action/back_up.hpp"

using namespace std::chrono_literals;

namespace nav2_behaviors
{

template<typename ActionT>
DriveOnHeading<ActionT>::DriveOnHeading()
: TimedBehavior<ActionT>(),
  feedback_(std::make_shared<typename ActionT::Feedback>())
{
}

template<typename ActionT>
DriveOnHeading<ActionT>::~DriveOnHeading()
{
}

template<typename ActionT>
void DriveOnHeading<ActionT>::onConfigure()
{
  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node,
    "simulate_ahead_time", rclcpp::ParameterValue(2.0));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);
}

template<typename ActionT>
Status DriveOnHeading<ActionT>::onRun(const std::shared_ptr<const typename ActionT::Goal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_INFO(
      this->logger_,
      "Backing up in Y and Z not supported, will only move in X.");
  }

  // Ensure that both the speed and direction have the same sign
  if (!((command->target.x > 0.0) == (command->speed > 0.0)) ) {
    RCLCPP_ERROR(this->logger_, "Speed and command sign did not match");
    return Status::FAILED;
  }

  command_x_ = command->target.x;
  command_speed_ = command->speed;
  command_time_allowance_ = command->time_allowance;

  end_time_ = this->steady_clock_.now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *this->tf_, this->global_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    RCLCPP_ERROR(this->logger_, "Initial robot pose is not available.");
    return Status::FAILED;
  }

  return Status::SUCCEEDED;
}

template<typename ActionT>
Status DriveOnHeading<ActionT>::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - this->steady_clock_.now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_,
      "Exceeded time allowance before reaching the DriveOnHeading goal - Exiting DriveOnHeading");
    return Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
    return Status::FAILED;
  }

  double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  double distance = sqrt(diff_x * diff_x + diff_y * diff_y);

  feedback_->distance_traveled = distance;
  this->action_server_->publish_feedback(feedback_);

  if (distance >= std::fabs(command_x_)) {
    this->stopRobot();
    return Status::SUCCEEDED;
  }

  // TODO(mhpanah): cmd_vel value should be passed as a parameter
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.y = 0.0;
  cmd_vel->angular.z = 0.0;
  cmd_vel->linear.x = command_speed_;

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(distance, cmd_vel.get(), pose2d)) {
    this->stopRobot();
    RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting DriveOnHeading");
    return Status::FAILED;
  }

  this->vel_pub_->publish(std::move(cmd_vel));

  return Status::RUNNING;
}

template<typename ActionT>
bool DriveOnHeading<ActionT>::isCollisionFree(
  const double & distance,
  geometry_msgs::msg::Twist * cmd_vel,
  geometry_msgs::msg::Pose2D & pose2d)
{
  // Simulate ahead by simulate_ahead_time_ in this->cycle_frequency_ increments
  int cycle_count = 0;
  double sim_position_change;
  const double diff_dist = abs(command_x_) - distance;
  const int max_cycle_count = static_cast<int>(this->cycle_frequency_ * simulate_ahead_time_);
  geometry_msgs::msg::Pose2D init_pose = pose2d;
  bool fetch_data = true;

  while (cycle_count < max_cycle_count) {
    sim_position_change = cmd_vel->linear.x * (cycle_count / this->cycle_frequency_);
    pose2d.x = init_pose.x + sim_position_change * cos(init_pose.theta);
    pose2d.y = init_pose.y + sim_position_change * sin(init_pose.theta);
    cycle_count++;

    if (diff_dist - abs(sim_position_change) <= 0.) {
      break;
    }

    if (!this->collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }
  return true;
}

Status BackUp::onRun(const std::shared_ptr<const BackUpAction::Goal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_INFO(
      logger_,
      "Backing up in Y and Z not supported, will only move in X.");
  }

  // Silently ensure that both the speed and direction are negative.
  command_x_ = -std::fabs(command->target.x);
  command_speed_ = -std::fabs(command->speed);
  command_time_allowance_ = command->time_allowance;

  end_time_ = steady_clock_.now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return Status::FAILED;
  }

  return Status::SUCCEEDED;
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::DriveOnHeading<>, nav2_core::Behavior)
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::BackUp, nav2_core::Behavior)
