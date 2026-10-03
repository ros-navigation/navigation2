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

PIDParameterHandler::PIDParameterHandler(
  const nav2::LifecycleNode::SharedPtr & node, const std::string & name,
  const rclcpp::Logger & logger)
: nav2_util::ParameterHandler<PIDParameters>(node, logger), name_(name)
{
  params_.x.kp = node->declare_or_get_parameter(name_ + ".kp_x", 1.0);
  params_.x.ki = node->declare_or_get_parameter(name_ + ".ki_x", 0.0);
  params_.x.kd = node->declare_or_get_parameter(name_ + ".kd_x", 0.0);
  params_.x.i_clamp = node->declare_or_get_parameter(name_ + ".i_clamp_x", 0.2);

  params_.y.kp = node->declare_or_get_parameter(name_ + ".kp_y", 2.2);
  params_.y.ki = node->declare_or_get_parameter(name_ + ".ki_y", 0.0);
  params_.y.kd = node->declare_or_get_parameter(name_ + ".kd_y", 0.0);
  params_.y.i_clamp = node->declare_or_get_parameter(name_ + ".i_clamp_y", 0.5);

  params_.theta.kp = node->declare_or_get_parameter(name_ + ".kp_theta", 1.2);
  params_.theta.ki = node->declare_or_get_parameter(name_ + ".ki_theta", 0.0);
  params_.theta.kd = node->declare_or_get_parameter(name_ + ".kd_theta", 0.0);
  params_.theta.i_clamp = node->declare_or_get_parameter(name_ + ".i_clamp_theta", 0.5);

  params_.v_linear_max = node->declare_or_get_parameter(name_ + ".v_linear_max", 0.25);
  params_.v_angular_max = node->declare_or_get_parameter(name_ + ".v_angular_max", 0.75);
  params_.lookahead_distance = node->declare_or_get_parameter(
    name_ + ".lookahead_distance", 0.25);
}

rcl_interfaces::msg::SetParametersResult PIDParameterHandler::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(name_ + ".") != 0) {
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

void PIDParameterHandler::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + ".kp_x") {
        params_.x.kp = parameter.as_double();
      } else if (param_name == name_ + ".ki_x") {
        params_.x.ki = parameter.as_double();
      } else if (param_name == name_ + ".kd_x") {
        params_.x.kd = parameter.as_double();
      } else if (param_name == name_ + ".i_clamp_x") {
        params_.x.i_clamp = parameter.as_double();
      } else if (param_name == name_ + ".kp_y") {
        params_.y.kp = parameter.as_double();
      } else if (param_name == name_ + ".ki_y") {
        params_.y.ki = parameter.as_double();
      } else if (param_name == name_ + ".kd_y") {
        params_.y.kd = parameter.as_double();
      } else if (param_name == name_ + ".i_clamp_y") {
        params_.y.i_clamp = parameter.as_double();
      } else if (param_name == name_ + ".kp_theta") {
        params_.theta.kp = parameter.as_double();
      } else if (param_name == name_ + ".ki_theta") {
        params_.theta.ki = parameter.as_double();
      } else if (param_name == name_ + ".kd_theta") {
        params_.theta.kd = parameter.as_double();
      } else if (param_name == name_ + ".i_clamp_theta") {
        params_.theta.i_clamp = parameter.as_double();
      } else if (param_name == name_ + ".v_linear_max") {
        params_.v_linear_max = parameter.as_double();
      } else if (param_name == name_ + ".v_angular_max") {
        params_.v_angular_max = parameter.as_double();
      } else if (param_name == name_ + ".lookahead_distance") {
        params_.lookahead_distance = parameter.as_double();
      }
    }
  }
  updated_ = true;
}

void PIDController::onConfigure(const nav2::LifecycleNode::SharedPtr & node)
{
  pid_param_handler_ = std::make_unique<PIDParameterHandler>(node, name_, logger_);
  pid_params_ = pid_param_handler_->getParams();
  {
    std::lock_guard<std::mutex> lock(pid_param_handler_->getMutex());
    applyParameters();
  }
  reset();
}

void PIDController::cleanup()
{
  ControllerBase::cleanup();
  pid_params_ = nullptr;
  pid_param_handler_.reset();
}

void PIDController::activate()
{
  ControllerBase::activate();
  pid_param_handler_->activate();
}

void PIDController::deactivate()
{
  ControllerBase::deactivate();
  pid_param_handler_->deactivate();
}

void PIDController::applyParameters()
{
  x_ = pid_params_->x;
  y_ = pid_params_->y;
  theta_ = pid_params_->theta;
  v_linear_max_ = pid_params_->v_linear_max;
  v_angular_max_ = pid_params_->v_angular_max;
  lookahead_distance_ = std::max(kMinLookahead, pid_params_->lookahead_distance);
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
  {
    std::lock_guard<std::mutex> lock(pid_param_handler_->getMutex());
    if (pid_param_handler_->isUpdated()) {
      applyParameters();
    }
  }

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
