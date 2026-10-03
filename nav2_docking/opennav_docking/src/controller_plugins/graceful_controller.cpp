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
#include <vector>

#include "opennav_docking/controller_plugins/graceful_controller.hpp"

#include "nav2_ros_common/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

using rcl_interfaces::msg::ParameterType;

namespace opennav_docking
{

GracefulParameterHandler::GracefulParameterHandler(
  const nav2::LifecycleNode::SharedPtr & node, const std::string & name,
  const rclcpp::Logger & logger)
: nav2_util::ParameterHandler<GracefulParameters>(node, logger), name_(name)
{
  params_.k_phi = node->declare_or_get_parameter(name_ + ".k_phi", 3.0);
  params_.k_delta = node->declare_or_get_parameter(name_ + ".k_delta", 2.0);
  params_.beta = node->declare_or_get_parameter(name_ + ".beta", 0.4);
  params_.lambda = node->declare_or_get_parameter(name_ + ".lambda", 2.0);
  params_.v_linear_min = node->declare_or_get_parameter(name_ + ".v_linear_min", 0.1);
  params_.v_linear_max = node->declare_or_get_parameter(name_ + ".v_linear_max", 0.25);
  params_.v_angular_max = node->declare_or_get_parameter(name_ + ".v_angular_max", 0.75);
  params_.slowdown_radius = node->declare_or_get_parameter(name_ + ".slowdown_radius", 0.25);
  params_.deceleration_max = node->declare_or_get_parameter(name_ + ".deceleration_max", 2.5);
}

rcl_interfaces::msg::SetParametersResult
GracefulParameterHandler::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  double v_linear_min = params_.v_linear_min;
  double v_linear_max = params_.v_linear_max;
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
      if (param_name == name_ + ".v_linear_min") {
        v_linear_min = parameter.as_double();
      } else if (param_name == name_ + ".v_linear_max") {
        v_linear_max = parameter.as_double();
      }
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

void GracefulParameterHandler::updateParametersCallback(
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
      if (param_name == name_ + ".k_phi") {
        params_.k_phi = parameter.as_double();
      } else if (param_name == name_ + ".k_delta") {
        params_.k_delta = parameter.as_double();
      } else if (param_name == name_ + ".beta") {
        params_.beta = parameter.as_double();
      } else if (param_name == name_ + ".lambda") {
        params_.lambda = parameter.as_double();
      } else if (param_name == name_ + ".v_linear_min") {
        params_.v_linear_min = parameter.as_double();
      } else if (param_name == name_ + ".v_linear_max") {
        params_.v_linear_max = parameter.as_double();
      } else if (param_name == name_ + ".v_angular_max") {
        params_.v_angular_max = parameter.as_double();
      } else if (param_name == name_ + ".slowdown_radius") {
        params_.slowdown_radius = parameter.as_double();
      } else if (param_name == name_ + ".deceleration_max") {
        params_.deceleration_max = parameter.as_double();
      }
    }
  }
  updated_ = true;
}

void GracefulController::onConfigure(const nav2::LifecycleNode::SharedPtr & node)
{
  graceful_param_handler_ = std::make_unique<GracefulParameterHandler>(node, name_, logger_);
  graceful_params_ = graceful_param_handler_->getParams();

  control_law_ = std::make_unique<nav2_graceful_controller::SmoothControlLaw>(
    graceful_params_->k_phi, graceful_params_->k_delta, graceful_params_->beta,
    graceful_params_->lambda, graceful_params_->slowdown_radius,
    graceful_params_->deceleration_max, graceful_params_->v_linear_min,
    graceful_params_->v_linear_max, graceful_params_->v_angular_max);
}

void GracefulController::cleanup()
{
  ControllerBase::cleanup();
  control_law_.reset();
  graceful_params_ = nullptr;
  graceful_param_handler_.reset();
}

void GracefulController::activate()
{
  ControllerBase::activate();
  graceful_param_handler_->activate();
}

void GracefulController::deactivate()
{
  ControllerBase::deactivate();
  graceful_param_handler_->deactivate();
}

void GracefulController::applyParameters()
{
  control_law_->setCurvatureConstants(
    graceful_params_->k_phi, graceful_params_->k_delta, graceful_params_->beta,
    graceful_params_->lambda);
  control_law_->setSlowdownRadius(graceful_params_->slowdown_radius);
  control_law_->setMaxDeceleration(graceful_params_->deceleration_max);
  control_law_->setSpeedLimit(
    graceful_params_->v_linear_min, graceful_params_->v_linear_max,
    graceful_params_->v_angular_max);
}

geometry_msgs::msg::Twist GracefulController::computeCommand(
  const geometry_msgs::msg::Pose & target, bool reverse, double /*dt*/)
{
  {
    // Apply updated parameters
    std::lock_guard<std::mutex> lock(graceful_param_handler_->getMutex());
    if (graceful_param_handler_->isUpdated()) {
      applyParameters();
    }
  }
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
