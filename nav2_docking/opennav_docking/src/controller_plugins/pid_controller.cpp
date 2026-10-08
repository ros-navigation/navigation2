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
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "opennav_docking/controller_plugins/pid_controller.hpp"

#include "angles/angles.h"

#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.hpp"

using rcl_interfaces::msg::ParameterType;

namespace opennav_docking
{

void PIDController::onConfigure(const nav2::LifecycleNode::SharedPtr & node)
{
  x_.kp = node->declare_or_get_parameter(name_ + ".kp_x", 1.0);
  x_.ki = node->declare_or_get_parameter(name_ + ".ki_x", 0.0);
  x_.kd = node->declare_or_get_parameter(name_ + ".kd_x", 0.0);
  x_.i_clamp = node->declare_or_get_parameter(name_ + ".i_clamp_x", 0.2);

  y_.kp = node->declare_or_get_parameter(name_ + ".kp_y", 2.2);
  y_.ki = node->declare_or_get_parameter(name_ + ".ki_y", 0.0);
  y_.kd = node->declare_or_get_parameter(name_ + ".kd_y", 0.0);
  y_.i_clamp = node->declare_or_get_parameter(name_ + ".i_clamp_y", 0.5);

  theta_.kp = node->declare_or_get_parameter(name_ + ".kp_theta", 1.2);
  theta_.ki = node->declare_or_get_parameter(name_ + ".ki_theta", 0.0);
  theta_.kd = node->declare_or_get_parameter(name_ + ".kd_theta", 0.0);
  theta_.i_clamp = node->declare_or_get_parameter(name_ + ".i_clamp_theta", 0.5);

  v_linear_max_ = node->declare_or_get_parameter(name_ + ".v_linear_max", 0.25);
  v_angular_max_ = node->declare_or_get_parameter(name_ + ".v_angular_max", 0.75);
  lookahead_distance_ = std::max(
    kMinLookahead, node->declare_or_get_parameter(name_ + ".lookahead_distance", 0.25));

  reset();
}

void PIDController::activate()
{
  ControllerBase::activate();
  auto node = node_.lock();
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &PIDController::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &PIDController::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
}

void PIDController::deactivate()
{
  ControllerBase::deactivate();
  auto node = node_.lock();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
  on_set_params_handler_.reset();
}

rcl_interfaces::msg::SetParametersResult PIDController::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(name_ + ".") != 0 || param_type != ParameterType::PARAMETER_DOUBLE) {
      continue;
    }
    if (!validateSharedParameter(parameter)) {
      result.successful = false;
    } else if (parameter.as_double() < 0.0) {
      RCLCPP_WARN(
        logger_, "The value of parameter '%s' is incorrectly set to %f, "
        "it should be >=0. Ignoring parameter update.",
        param_name.c_str(), parameter.as_double());
      result.successful = false;
    }
  }
  return result;
}

void PIDController::updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(name_ + ".") != 0 || param_type != ParameterType::PARAMETER_DOUBLE) {
      continue;
    }
    if (updateSharedParameter(parameter)) {
      continue;
    }
    if (param_name == name_ + ".kp_x") {
      x_.kp = parameter.as_double();
    } else if (param_name == name_ + ".ki_x") {
      x_.ki = parameter.as_double();
    } else if (param_name == name_ + ".kd_x") {
      x_.kd = parameter.as_double();
    } else if (param_name == name_ + ".i_clamp_x") {
      x_.i_clamp = parameter.as_double();
    } else if (param_name == name_ + ".kp_y") {
      y_.kp = parameter.as_double();
    } else if (param_name == name_ + ".ki_y") {
      y_.ki = parameter.as_double();
    } else if (param_name == name_ + ".kd_y") {
      y_.kd = parameter.as_double();
    } else if (param_name == name_ + ".i_clamp_y") {
      y_.i_clamp = parameter.as_double();
    } else if (param_name == name_ + ".kp_theta") {
      theta_.kp = parameter.as_double();
    } else if (param_name == name_ + ".ki_theta") {
      theta_.ki = parameter.as_double();
    } else if (param_name == name_ + ".kd_theta") {
      theta_.kd = parameter.as_double();
    } else if (param_name == name_ + ".i_clamp_theta") {
      theta_.i_clamp = parameter.as_double();
    } else if (param_name == name_ + ".v_linear_max") {
      v_linear_max_ = parameter.as_double();
    } else if (param_name == name_ + ".v_angular_max") {
      v_angular_max_ = parameter.as_double();
    } else if (param_name == name_ + ".lookahead_distance") {
      lookahead_distance_ = std::max(kMinLookahead, parameter.as_double());
    }
  }
}

void PIDController::reset()
{
  x_.reset();
  y_.reset();
  theta_.reset();
}

double PIDController::evaluate(
  const ControllerState & channel, double error, double integral, double derivative)
{
  return channel.kp * error + channel.ki * integral + channel.kd * derivative;
}

double PIDController::advance(ControllerState & channel, double error, double dt)
{
  const double out = evaluate(channel, error, channel.integral, channel.derivative);

  channel.integral =
    std::clamp(channel.integral + error * dt, -channel.i_clamp, channel.i_clamp);
  channel.derivative = (error - channel.prev_error) / dt;
  channel.prev_error = error;
  return out;
}


void PIDController::poseError(
  const geometry_msgs::msg::Pose & target, bool reverse, double lookahead,
  double & rho, double & alpha, double & alignment)
{
  const double sign = reverse ? -1.0 : 1.0;
  const double e_x = sign * target.position.x;
  const double e_y = sign * target.position.y;

  rho = std::hypot(e_x, e_y);
  // Reverse direction if passed dock's axis
  alpha = std::atan2(e_y, std::max(std::abs(e_x), lookahead));
  if (e_x < 0.0) {
    rho = -rho;
    alpha = -alpha;
  }
  alignment = angles::normalize_angle(alpha - tf2::getYaw(target.orientation));
}

geometry_msgs::msg::Twist PIDController::computeCommand(
  const geometry_msgs::msg::Pose & target, bool reverse, double dt)
{
  double rho = 0.0, alpha = 0.0, alignment = 0.0;
  poseError(target, reverse, lookahead_distance_, rho, alpha, alignment);

  const double v_raw = advance(x_, rho, dt);
  const double w_bearing = advance(y_, alpha, dt);
  const double w_alignment = advance(theta_, alignment, dt);

  const double v = std::clamp(v_raw, -v_linear_max_, v_linear_max_);
  const double w = std::clamp(w_bearing + w_alignment, -v_angular_max_, v_angular_max_);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = (reverse ? -1.0 : 1.0) * v;
  cmd.angular.z = w;
  return cmd;
}

geometry_msgs::msg::Pose PIDController::predictNextPose(
  double dt, const geometry_msgs::msg::Pose & target,
  const geometry_msgs::msg::Pose & current, bool reverse)
{
  const double dx = target.position.x - current.position.x;
  const double dy = target.position.y - current.position.y;
  const double yaw = tf2::getYaw(current.orientation);

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  geometry_msgs::msg::Pose relative;
  relative.position.x = dx * cos_yaw + dy * sin_yaw;
  relative.position.y = -dx * sin_yaw + dy * cos_yaw;
  relative.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
    angles::shortest_angular_distance(yaw, tf2::getYaw(target.orientation)));

  double rho = 0.0, alpha = 0.0, alignment = 0.0;
  poseError(relative, reverse, lookahead_distance_, rho, alpha, alignment);

  const double v_raw = evaluate(x_, rho, x_.integral, x_.derivative);
  const double w_raw =
    evaluate(y_, alpha, y_.integral, y_.derivative) +
    evaluate(theta_, alignment, theta_.integral, theta_.derivative);

  const double v =
    (reverse ? -1.0 : 1.0) * std::clamp(v_raw, -v_linear_max_, v_linear_max_);
  const double w = std::clamp(w_raw, -v_angular_max_, v_angular_max_);

  geometry_msgs::msg::Pose next = current;
  next.position.x += v * cos_yaw * dt;
  next.position.y += v * sin_yaw * dt;
  next.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw + w * dt);
  return next;
}

}  // namespace opennav_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(opennav_docking::PIDController, opennav_docking::ControllerBase)
