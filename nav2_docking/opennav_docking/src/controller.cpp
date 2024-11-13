// Copyright (c) 2024 Open Navigation LLC
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

#include "rclcpp/rclcpp.hpp"
#include "opennav_docking/controller.hpp"
#include "nav2_util/node_utils.hpp"

namespace opennav_docking
{

Controller::Controller(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.k_phi", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.k_delta", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.beta", rclcpp::ParameterValue(0.4));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.lambda", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.v_linear_min", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.v_linear_max", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.v_angular_min", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.v_angular_max", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.slowdown_radius", rclcpp::ParameterValue(0.25));

  node->get_parameter("controller.k_phi", k_phi_);
  node->get_parameter("controller.k_delta", k_delta_);
  node->get_parameter("controller.beta", beta_);
  node->get_parameter("controller.lambda", lambda_);
  node->get_parameter("controller.v_linear_min", v_linear_min_);
  node->get_parameter("controller.v_linear_max", v_linear_max_);
  node->get_parameter("controller.v_angular_min", v_angular_min_);
  node->get_parameter("controller.v_angular_max", v_angular_max_);
  node->get_parameter("controller.slowdown_radius", slowdown_radius_);
  control_law_ = std::make_unique<nav2_graceful_controller::SmoothControlLaw>(
    k_phi_, k_delta_, beta_, lambda_, slowdown_radius_, v_linear_min_, v_linear_max_,
    v_angular_max_);

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&Controller::dynamicParametersCallback, this, std::placeholders::_1));
}

bool Controller::computeVelocityCommand(
  const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Pose & robot_pose,
  geometry_msgs::msg::Twist & cmd, bool backward)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  cmd = control_law_->calculateRegularVelocity(pose, robot_pose, backward);
  return true;
}

rcl_interfaces::msg::SetParametersResult
Controller::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller.k_phi") {
        k_phi_ = parameter.as_double();
      } else if (name == "controller.k_delta") {
        k_delta_ = parameter.as_double();
      } else if (name == "controller.beta") {
        beta_ = parameter.as_double();
      } else if (name == "controller.lambda") {
        lambda_ = parameter.as_double();
      } else if (name == "controller.v_linear_min") {
        v_linear_min_ = parameter.as_double();
      } else if (name == "controller.v_linear_max") {
        v_linear_max_ = parameter.as_double();
      } else if (name == "controller.v_angular_max") {
        v_angular_max_ = parameter.as_double();
      } else if (name == "controller.v_angular_min") {
        v_angular_min_ = parameter.as_double();
      } else if (name == "controller.slowdown_radius") {
        slowdown_radius_ = parameter.as_double();
      }

      // Update the smooth control law with the new params
      control_law_->setCurvatureConstants(k_phi_, k_delta_, beta_, lambda_);
      control_law_->setSlowdownRadius(slowdown_radius_);
      control_law_->setSpeedLimit(v_linear_min_, v_linear_max_, v_angular_max_);
    }
  }

  result.successful = true;
  return result;
}

geometry_msgs::msg::Twist Controller::rotateToTarget(const double & angle_to_target)
{
  geometry_msgs::msg::Twist vel;
  vel.linear.x = 0.0;
  vel.angular.z = 0.0;
  if(angle_to_target > 0) {
    vel.angular.z = std::clamp(1.0 * angle_to_target * v_angular_max_,
    v_angular_min_, v_angular_max_);
  } else if (angle_to_target < 0) {
    vel.angular.z = std::clamp(1.0 * angle_to_target * v_angular_max_,
    -v_angular_max_, -v_angular_min_);
  }
  return vel;
}


}  // namespace opennav_docking
