// Copyright (c) 2022 Samsung Research
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
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_velocity_smoother/velocity_smoother.hpp"

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;

namespace nav2_velocity_smoother
{

VelocitySmoother::VelocitySmoother(const rclcpp::NodeOptions & options)
: LifecycleNode("velocity_smoother", "", false, options)
{
}

VelocitySmoother::~VelocitySmoother()
{
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
}

nav2_util::CallbackReturn
VelocitySmoother::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring velocity smoother");

  auto node = shared_from_this();
  // TODO angular velocities / acelerations / deadband (make params vectors)

  // Smoothing metadata
  std::string feedback_type;
  declare_parameter_if_not_declared("smoothing_frequency", rclcpp::ParameterValue(50));
  declare_parameter_if_not_declared("feedback_type", rclcpp::ParameterType::PARAMETER_STRING);

  // Kinematics TODO jerk
  declare_parameter_if_not_declared("max_velocity", rclcpp::ParameterValue(0.50));
  declare_parameter_if_not_declared("min_velocity", rclcpp::ParameterValue(-0.50));
  declare_parameter_if_not_declared("max_accel", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared("min_accel", rclcpp::ParameterValue(-1.5));

  // Get feature parameters
  declare_parameter_if_not_declared("deadband_velocity", rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared("velocity_timeout", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared("odom_topic", rclcpp::ParameterValue("odom"));
  declare_parameter_if_not_declared("odom_duration", rclcpp::ParameterValue(0.3));

  node->get_parameter("smoothing_frequency", smoothing_frequency_);
  node->get_parameter("feedback_type", feedback_type);

  node->get_parameter("max_velocity", max_velocity_);
  node->get_parameter("min_velocity", min_velocity_);
  node->get_parameter("max_accel", max_accel_);
  node->get_parameter("min_accel", min_accel_);

  std::string odom_topic;
  double odom_duration;
  double velocity_timeout_dbl;
  node->get_parameter("deadband_velocity", deadband_velocity_); // TODO vector too
  node->get_parameter("velocity_timeout", velocity_timeout_dbl);
  node->get_parameter("odom_topic", odom_topic);
  node->get_parameter("odom_duration", odom_duration);

  // Convert velocity timeout to duration
  velocity_timeout_ = rclpp::Duration::from_seconds(velocity_timeout_dbl);

  // Get control type
  std::transform(feedback_type.begin(), feedback_type.end(), feedback_type.begin(), ::toupper);
  if (feedback_type == "OPEN_LOOP") {
    open_loop_ = true;
  } else if (feedback_type == "CLOSED_LOOP") {
    open_loop_ = false;
  } else {
    throw std::runtime_error("Invalid feedback_type, options are OPEN_LOOP and CLOSED_LOOP.");
  }

  // Setup interfaces based on feedback type
  if (!open_loop_) {
    odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(node, odom_duration, odom_topic);
  }

  // Setup inputs / outputs
  smoothed_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("smoothed_cmd_vel", 1);
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(1),
    std::bind(&VelocitySmoother::inputCommandCallback, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VelocitySmoother::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  vel_publisher_->on_activate();

  // Create worker timer
  timer_ = create_wall_timer(
    rclcpp::Duration::from_seconds(1.0 / smoothing_frequency_),
    std::bind(&VelocitySmoother::smootherTimer, this));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VelocitySmoother::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  vel_publisher_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VelocitySmoother::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  vel_publisher_.reset();
  odom_smoother_.reset();
  cmd_sub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VelocitySmoother::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void inputCommandCallback()
{
  // TODO
  command_ = msg;
}

void smootherTimer()  // TODO
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->header = command_->header;

  // Check for velocity timeout
  if (now() - command_.header.stamp > velocity_timeout_) {
    // publish 0 speed, nothing received, danger TODO
    smoothed_cmd_pub_->publish(std::move(cmd_vel));
    return;
  }

  // Get current velocity based on feedback type
  geometry_msgs::msg::Twist current_;
  if (open_loop_) {
    current_ = last_cmd_;  //TODO what if pause long time between or initailized?
  } else {
    current_ = odom_smoother_->getTwist();
  }

  // This lib does not work properly when angles wrap, so we need to unwind the path first TODO
  // Smooth the velocity
  Ruckig<3> smoother {1.0 / smoothing_frequency_};
  InputParameter<3> input;
  OutputParameter<3> output;

  input.control_interface = ControlInterface::Velocity;
  iput.delta_time = 1.0 / smoothing_frequency_;

  input.current_velocity = {current_->linear.x, current_->linear.y, current_->angular.z};
  input.current_acceleration = {0.0, 2.5, -0.5};// TODO? open loop only?

  input.target_velocity = {command_->linear.x, command_->linear.y, command_->angular.z};
  input.target_acceleration = {0.0, 0.0, 0.5};// TODO? open loop only?
  
  input.max_velocity = {max_velocity_, max_velocity_, max_velocity_};
  input.max_acceleration = {max_accel_, max_accel_, max_accel_};
  input.max_jerk = {6.0, 6.0, 4.0}; // TODO?

  input.min_velocity = {min_velocity_, min_velocity_, min_velocity_};
  input.min_acceleration = {min_accel_, min_accel_, min_accel_};
  input.min_jerk = {6.0, 6.0, 4.0};// TODO?


  // minimum_duration? target/current position?

  output.pass_to_input(input);  //TODO?
  auto state = smoother->update(input, output);
  
  if (state == ruckig::Result::Working || state == ruckig::Result::Finished) {
    // Apply smoothed output with deadbands

    // TODO are new_velocity at the minimum duration time? How to get at time I'm interested in (1/hz)
    cmd_vel.linear.x = output.new_velocity[0] < deadband_velocity_ ? 0.0 : output.new_velocity[0];
    cmd_vel.linear.y = output.new_velocity[1] < deadband_velocity_ ? 0.0 : output.new_velocity[1];
    cmd_vel.angular.z = output.new_velocity[2] < deadband_velocity_ ? 0.0 : output.new_velocity[2];
    output.pass_to_input(input);  // TODO? only if iterate several times with same objects
  } else {
    // TODO error state?
  }

  if (open_loop_) {
    last_cmd_ = *cmd_vel;
  }

  smoothed_cmd_pub_->publish(std::move(cmd_vel));
}

}  // namespace nav2_velocity_smoother


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_velocity_smoother::VelocitySmoother)
