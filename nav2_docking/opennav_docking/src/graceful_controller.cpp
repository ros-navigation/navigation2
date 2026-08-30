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

#include <memory>
#include <string>

#include "opennav_docking/graceful_controller.hpp"

#include "nav2_ros_common/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

using rcl_interfaces::msg::ParameterType;

namespace opennav_docking
{

void GracefulController::configureController(const nav2::LifecycleNode::SharedPtr & node)
{
  k_phi_ = node->declare_or_get_parameter(name_ + ".k_phi", 3.0);
  k_delta_ = node->declare_or_get_parameter(name_ + ".k_delta", 2.0);
  beta_ = node->declare_or_get_parameter(name_ + ".beta", 0.4);
  lambda_ = node->declare_or_get_parameter(name_ + ".lambda", 2.0);
  v_linear_min_ = node->declare_or_get_parameter(name_ + ".v_linear_min", 0.1);
  v_linear_max_ = node->declare_or_get_parameter(name_ + ".v_linear_max", 0.25);
  v_angular_max_ = node->declare_or_get_parameter(name_ + ".v_angular_max", 0.75);
  slowdown_radius_ = node->declare_or_get_parameter(name_ + ".slowdown_radius", 0.25);
  deceleration_max_ = node->declare_or_get_parameter(name_ + ".deceleration_max", 2.5);

  control_law_ = std::make_unique<nav2_graceful_controller::SmoothControlLaw>(
    k_phi_, k_delta_, beta_, lambda_, slowdown_radius_, deceleration_max_,
    v_linear_min_, v_linear_max_, v_angular_max_);
}

void GracefulController::cleanup()
{
  ControllerBase::cleanup();
  control_law_.reset();
}

geometry_msgs::msg::Twist GracefulController::computeCommand(
  const geometry_msgs::msg::Pose & target, bool reverse, double /*dt*/)
{
  return control_law_->calculateRegularVelocity(target, reverse);
}

geometry_msgs::msg::Pose GracefulController::predictNextPose(
  double dt, const geometry_msgs::msg::Pose & target,
  const geometry_msgs::msg::Pose & current, bool reverse)
{
  return control_law_->calculateNextPose(dt, target, current, reverse);
}

void GracefulController::updateParameter(
  const std::string & name, const rclcpp::Parameter & parameter)
{
  ControllerBase::updateParameter(name, parameter);

  if (parameter.get_type() != ParameterType::PARAMETER_DOUBLE) {
    return;
  }

  if (name == "k_phi") {
    k_phi_ = parameter.as_double();
  } else if (name == "k_delta") {
    k_delta_ = parameter.as_double();
  } else if (name == "beta") {
    beta_ = parameter.as_double();
  } else if (name == "lambda") {
    lambda_ = parameter.as_double();
  } else if (name == "v_linear_min") {
    v_linear_min_ = parameter.as_double();
  } else if (name == "v_linear_max") {
    v_linear_max_ = parameter.as_double();
  } else if (name == "v_angular_max") {
    v_angular_max_ = parameter.as_double();
  } else if (name == "slowdown_radius") {
    slowdown_radius_ = parameter.as_double();
  } else if (name == "deceleration_max") {
    deceleration_max_ = parameter.as_double();
  }

  // Update the smooth control law with the new params
  control_law_->setCurvatureConstants(k_phi_, k_delta_, beta_, lambda_);
  control_law_->setSlowdownRadius(slowdown_radius_);
  control_law_->setMaxDeceleration(deceleration_max_);
  control_law_->setSpeedLimit(v_linear_min_, v_linear_max_, v_angular_max_);
}

}  // namespace opennav_docking
