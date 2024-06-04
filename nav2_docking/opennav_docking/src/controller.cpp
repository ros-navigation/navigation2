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
    node, "controller.v_angular_max", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.slowdown_radius", rclcpp::ParameterValue(0.25));

  double k_phi, k_delta, beta, lambda;
  double slowdown_radius, v_linear_min, v_linear_max, v_angular_max;
  node->get_parameter("controller.k_phi", k_phi);
  node->get_parameter("controller.k_delta", k_delta);
  node->get_parameter("controller.beta", beta);
  node->get_parameter("controller.lambda", lambda);
  node->get_parameter("controller.v_linear_min", v_linear_min);
  node->get_parameter("controller.v_linear_max", v_linear_max);
  node->get_parameter("controller.v_angular_max", v_angular_max);
  node->get_parameter("controller.slowdown_radius", slowdown_radius);
  control_law_ = std::make_unique<nav2_graceful_controller::SmoothControlLaw>(
    k_phi, k_delta, beta, lambda, slowdown_radius, v_linear_min, v_linear_max, v_angular_max);
}

bool Controller::computeVelocityCommand(
  const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & cmd, bool backward)
{
  cmd = control_law_->calculateRegularVelocity(pose, backward);
  return true;
}

}  // namespace opennav_docking
