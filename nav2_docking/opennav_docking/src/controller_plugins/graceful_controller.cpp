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

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "opennav_docking/controller_plugins/graceful_controller.hpp"

#include "nav2_ros_common/node_utils.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"

using rcl_interfaces::msg::ParameterType;

namespace opennav_docking
{

void GracefulController::onConfigure(const nav2::LifecycleNode::SharedPtr & node)
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

void GracefulController::activate()
{
  ControllerBase::activate();
  auto node = node_.lock();
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &GracefulController::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &GracefulController::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
}

void GracefulController::deactivate()
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

rcl_interfaces::msg::SetParametersResult GracefulController::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  double v_linear_min = v_linear_min_;
  double v_linear_max = v_linear_max_;
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
    if (param_name == name_ + ".v_linear_min") {
      v_linear_min = parameter.as_double();
    } else if (param_name == name_ + ".v_linear_max") {
      v_linear_max = parameter.as_double();
    }
  }
  if (v_linear_min > v_linear_max) {
    RCLCPP_WARN(
      logger_, "v_linear_min (%f) of %s exceeds v_linear_max (%f), rejecting parameter change.",
      v_linear_min, name_.c_str(), v_linear_max);
    result.successful = false;
  }
  return result;
}

void GracefulController::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
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
    if (param_name == name_ + ".k_phi") {
      k_phi_ = parameter.as_double();
    } else if (param_name == name_ + ".k_delta") {
      k_delta_ = parameter.as_double();
    } else if (param_name == name_ + ".beta") {
      beta_ = parameter.as_double();
    } else if (param_name == name_ + ".lambda") {
      lambda_ = parameter.as_double();
    } else if (param_name == name_ + ".v_linear_min") {
      v_linear_min_ = parameter.as_double();
    } else if (param_name == name_ + ".v_linear_max") {
      v_linear_max_ = parameter.as_double();
    } else if (param_name == name_ + ".v_angular_max") {
      v_angular_max_ = parameter.as_double();
    } else if (param_name == name_ + ".slowdown_radius") {
      slowdown_radius_ = parameter.as_double();
    } else if (param_name == name_ + ".deceleration_max") {
      deceleration_max_ = parameter.as_double();
    }
  }

  // Update the smooth control law with the new params
  control_law_->setCurvatureConstants(k_phi_, k_delta_, beta_, lambda_);
  control_law_->setSlowdownRadius(slowdown_radius_);
  control_law_->setMaxDeceleration(deceleration_max_);
  control_law_->setSpeedLimit(v_linear_min_, v_linear_max_, v_angular_max_);
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

}  // namespace opennav_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(opennav_docking::GracefulController, opennav_docking::ControllerBase)
