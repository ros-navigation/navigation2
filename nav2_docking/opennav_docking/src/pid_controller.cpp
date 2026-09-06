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
#include <string>

#include "opennav_docking/pid_controller.hpp"

#include "angles/angles.h"

#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.hpp"

using rcl_interfaces::msg::ParameterType;

namespace opennav_docking
{

void PIDController::configureController(const nav2::LifecycleNode::SharedPtr & node)
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
  alpha = std::atan2(e_y, std::max(e_x, lookahead));
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

void PIDController::updateParameter(
  const std::string & name, const rclcpp::Parameter & parameter)
{
  ControllerBase::updateParameter(name, parameter);

  if (parameter.get_type() != ParameterType::PARAMETER_DOUBLE) {
    return;
  }
  const double value = parameter.as_double();

  if (name == "kp_x") {
    x_.kp = value;
  } else if (name == "ki_x") {
    x_.ki = value;
  } else if (name == "kd_x") {
    x_.kd = value;
  } else if (name == "i_clamp_x") {
    x_.i_clamp = value;
  } else if (name == "kp_y") {
    y_.kp = value;
  } else if (name == "ki_y") {
    y_.ki = value;
  } else if (name == "kd_y") {
    y_.kd = value;
  } else if (name == "i_clamp_y") {
    y_.i_clamp = value;
  } else if (name == "kp_theta") {
    theta_.kp = value;
  } else if (name == "ki_theta") {
    theta_.ki = value;
  } else if (name == "kd_theta") {
    theta_.kd = value;
  } else if (name == "i_clamp_theta") {
    theta_.i_clamp = value;
  } else if (name == "v_linear_max") {
    v_linear_max_ = value;
  } else if (name == "v_angular_max") {
    v_angular_max_ = value;
  } else if (name == "lookahead_distance") {
    lookahead_distance_ = std::max(kMinLookahead, value);
  }
}

}  // namespace opennav_docking
