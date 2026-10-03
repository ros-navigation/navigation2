// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2026 Karinca Robotics
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

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "opennav_following/controller.hpp"

#include "rclcpp/rclcpp.hpp"

using rcl_interfaces::msg::ParameterType;

namespace opennav_following
{

Controller::Controller(const nav2::LifecycleNode::SharedPtr & node)
{
  logger_ = node->get_logger();

  k_phi_ = node->declare_or_get_parameter("controller.k_phi", 3.0);
  k_delta_ = node->declare_or_get_parameter("controller.k_delta", 2.0);
  beta_ = node->declare_or_get_parameter("controller.beta", 0.4);
  lambda_ = node->declare_or_get_parameter("controller.lambda", 2.0);
  v_linear_min_ = node->declare_or_get_parameter("controller.v_linear_min", 0.1);
  v_linear_max_ = node->declare_or_get_parameter("controller.v_linear_max", 0.25);
  v_angular_max_ = node->declare_or_get_parameter("controller.v_angular_max", 0.75);
  slowdown_radius_ = node->declare_or_get_parameter("controller.slowdown_radius", 0.25);
  deceleration_max_ = node->declare_or_get_parameter("controller.deceleration_max", 2.5);
  rotate_to_heading_angular_vel_ = node->declare_or_get_parameter(
    "controller.rotate_to_heading_angular_vel", 1.0);
  rotate_to_heading_max_angular_accel_ = node->declare_or_get_parameter(
    "controller.rotate_to_heading_max_angular_accel", 3.2);

  control_law_ = std::make_unique<nav2_graceful_controller::SmoothControlLaw>(
    k_phi_, k_delta_, beta_, lambda_, slowdown_radius_, deceleration_max_,
    v_linear_min_, v_linear_max_, v_angular_max_);

  // Add callback for dynamic parameters
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &Controller::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &Controller::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
}

Controller::~Controller()
{
  control_law_.reset();
}

geometry_msgs::msg::Twist Controller::computeVelocityCommands(
  const geometry_msgs::msg::Pose & pose, bool backward)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  return control_law_->calculateRegularVelocity(pose, backward);
}

geometry_msgs::msg::Twist Controller::computeRotateToHeadingCommand(
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::Twist & current_velocity,
  const double & dt)
{
  geometry_msgs::msg::Twist cmd_vel;
  const double sign = angular_distance_to_heading > 0.0 ? 1.0 : -1.0;
  const double angular_vel = sign * rotate_to_heading_angular_vel_;
  const double min_feasible_angular_speed =
    current_velocity.angular.z - rotate_to_heading_max_angular_accel_ * dt;
  const double max_feasible_angular_speed =
    current_velocity.angular.z + rotate_to_heading_max_angular_accel_ * dt;
  cmd_vel.angular.z =
    std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

  // Check if we need to slow down to avoid overshooting
  double max_vel_to_stop =
    std::sqrt(2 * rotate_to_heading_max_angular_accel_ * fabs(angular_distance_to_heading));
  if (fabs(cmd_vel.angular.z) > max_vel_to_stop) {
    cmd_vel.angular.z = sign * max_vel_to_stop;
  }

  return cmd_vel;
}

rcl_interfaces::msg::SetParametersResult Controller::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find("controller.") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (parameter.as_double() < 0.0) {
        RCLCPP_WARN(
        logger_, "The value of parameter '%s' is incorrectly set to %f, "
        "it should be >=0. Ignoring parameter update.",
        param_name.c_str(), parameter.as_double());
        result.successful = false;
      }
    }
  }
  return result;
}

void
Controller::updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find("controller.") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "controller.k_phi") {
        k_phi_ = parameter.as_double();
      } else if (param_name == "controller.k_delta") {
        k_delta_ = parameter.as_double();
      } else if (param_name == "controller.beta") {
        beta_ = parameter.as_double();
      } else if (param_name == "controller.lambda") {
        lambda_ = parameter.as_double();
      } else if (param_name == "controller.v_linear_min") {
        v_linear_min_ = parameter.as_double();
      } else if (param_name == "controller.v_linear_max") {
        v_linear_max_ = parameter.as_double();
      } else if (param_name == "controller.v_angular_max") {
        v_angular_max_ = parameter.as_double();
      } else if (param_name == "controller.slowdown_radius") {
        slowdown_radius_ = parameter.as_double();
      } else if (param_name == "controller.deceleration_max") {
        deceleration_max_ = parameter.as_double();
      } else if (param_name == "controller.rotate_to_heading_angular_vel") {
        rotate_to_heading_angular_vel_ = parameter.as_double();
      } else if (param_name == "controller.rotate_to_heading_max_angular_accel") {
        rotate_to_heading_max_angular_accel_ = parameter.as_double();
      }

      // Update the smooth control law with the new params
      control_law_->setCurvatureConstants(k_phi_, k_delta_, beta_, lambda_);
      control_law_->setSlowdownRadius(slowdown_radius_);
      control_law_->setMaxDeceleration(deceleration_max_);
      control_law_->setSpeedLimit(v_linear_min_, v_linear_max_, v_angular_max_);
    }
  }
}

}  // namespace opennav_following
