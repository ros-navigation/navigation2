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

#include <utility>

#include "assisted_teleop.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behaviors
{
AssistedTeleop::AssistedTeleop()
: TimedBehavior<AssistedTeleopAction>(),
  feedback_(std::make_shared<AssistedTeleopAction::Feedback>())
{}

void AssistedTeleop::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // set up parameters
  nav2_util::declare_parameter_if_not_declared(
    node,
    "projection_time", rclcpp::ParameterValue(1.0));

  nav2_util::declare_parameter_if_not_declared(
    node,
    "input_vel_topic", rclcpp::ParameterValue(std::string("input_vel_topic")));

  nav2_util::declare_parameter_if_not_declared(
    node,
    "joystick_topic", rclcpp::ParameterValue(std::string("joystick_topic")));

  node->get_parameter("projection_time", projection_time_);
  node->get_parameter("input_vel_topic", input_vel_topic_);
  node->get_parameter("joystick_topic", joystick_topic_);
}

Status AssistedTeleop::onRun(const std::shared_ptr<const AssistedTeleopAction::Goal> command)
{
  command_time_allowance_ = command->time_allowance;
  end_time_ = steady_clock_.now() + command_time_allowance_;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    input_vel_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(
      &AssistedTeleop::inputVelocityCallback,
      this, std::placeholders::_1));

  joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>(
    joystick_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(
      &AssistedTeleop::joyCallback,
      this, std::placeholders::_1));

  return Status::SUCCEEDED;
}

void AssistedTeleop::onActionCompletion()
{
  RCLCPP_INFO(logger_, "Action completed");
  input_twist_ = geometry_msgs::msg::Twist();
  vel_sub_.reset();
  joy_sub_.reset();
}

Status AssistedTeleop::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - steady_clock_.now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "Exceeded time allowance before reaching the BackUp goal - Exiting BackUp");
    return Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return Status::FAILED;
  }

  // user states that teleop was successful
  if (joy_.buttons.size() > 0 && joy_.buttons[0] > 0) {
    stopRobot();
    return Status::SUCCEEDED;
  }

  // user states that teleop failed
  if (joy_.buttons.size() > 1 && joy_.buttons[1] > 0) {
    stopRobot();
    return Status::FAILED;
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel = computeVelocity(input_twist_);
  vel_pub_->publish(std::move(cmd_vel));

  return Status::RUNNING;
}

geometry_msgs::msg::Twist AssistedTeleop::computeVelocity(geometry_msgs::msg::Twist & twist)
{
  // TODO(jwallace42): Waiting for safety node to be completed.
  return twist;
}

geometry_msgs::msg::Pose2D AssistedTeleop::projectPose(
  geometry_msgs::msg::Pose2D pose,
  geometry_msgs::msg::Twist twist,
  double projection_time)
{
  geometry_msgs::msg::Pose2D projected_pose;

  projected_pose.x = projection_time * (
    twist.linear.x * cos(pose.theta) +
    twist.linear.y * sin(pose.theta));

  projected_pose.y = projection_time * (
    twist.linear.x * sin(pose.theta) -
    twist.linear.y * cos(pose.theta));

  projected_pose.theta = projection_time * twist.angular.z;

  return projected_pose;
}

void AssistedTeleop::inputVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_INFO(logger_, "Hit input Vel callback");
  input_twist_.linear.x = msg->linear.x;
  input_twist_.linear.y = msg->linear.y;
  input_twist_.angular.z = msg->angular.z;
}

void AssistedTeleop::joyCallback(const sensor_msgs::msg::Joy msg)
{
  RCLCPP_INFO(logger_, "Hit joy callback");
  joy_.buttons = msg.buttons;
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::AssistedTeleop, nav2_core::Behavior)
