// Copyright (c) 2022 Samsung Research America
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
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav2_controller/parameter_handler.hpp"

namespace nav2_controller
{

using nav2::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  nav2::LifecycleNode::SharedPtr node,
  const rclcpp::Logger & logger,
  const double costmap_size_x)
{
  node_ = node;
  logger_ = logger;

  declare_parameter_if_not_declared(
    node, "controller_frequency", rclcpp::ParameterValue(20.0));
  declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));

  declare_parameter_if_not_declared(
    node, "min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter_if_not_declared(
    node, "min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter_if_not_declared(
    node, "min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));

  declare_parameter_if_not_declared(
    node, "speed_limit_topic", rclcpp::ParameterValue("speed_limit"));

  declare_parameter_if_not_declared(
    node, "failure_tolerance", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, "use_realtime_priority", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, "publish_zero_velocity", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, "costmap_update_timeout", rclcpp::ParameterValue(0.30));

  declare_parameter_if_not_declared(
    node, "odom_topic", rclcpp::ParameterValue("odom"));
  declare_parameter_if_not_declared(
    node, "odom_duration", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, "interpolate_curvature_after_goal", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, "max_robot_pose_search_dist", rclcpp::ParameterValue(costmap_size_x));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enforce_path_inversion", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".inversion_xy_tolerance", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".inversion_yaw_tolerance", rclcpp::ParameterValue(0.4));

  node->get_parameter("controller_frequency", params_.controller_frequency);
  node->get_parameter("transform_tolerance", params_.transform_tolerance);
  node->get_parameter("min_x_velocity_threshold", params_.min_x_velocity_threshold);
  node->get_parameter("min_y_velocity_threshold", params_.min_y_velocity_threshold);
  node->get_parameter("min_theta_velocity_threshold", params_.min_theta_velocity_threshold);

  RCLCPP_INFO(
    node->get_logger(),
    "Controller frequency set to %.4fHz",
    params_.controller_frequency);

  node->get_parameter("speed_limit_topic", params_.speed_limit_topic);
  node->get_parameter("odom_topic", params_.odom_topic);
  node->get_parameter("odom_duration", params_.odom_duration);

  node->get_parameter("failure_tolerance", params_.failure_tolerance);
  node->get_parameter("use_realtime_priority", params_.use_realtime_priority);
  node->get_parameter("publish_zero_velocity", params_.publish_zero_velocity);
  node->get_parameter(
    "interpolate_curvature_after_goal",
    params_.interpolate_curvature_after_goal);
  node->get_parameter("max_robot_pose_search_dist", params_.max_robot_pose_search_dist);
  node->get_parameter("costmap_update_timeout", params_.costmap_update_timeout);
  node->get_parameter(plugin_name_ + ".enforce_path_inversion", params_.enforce_path_inversion);
  node->get_parameter(plugin_name_ + ".inversion_xy_tolerance", params_.inversion_xy_tolerance);
  node->get_parameter(plugin_name_ + ".inversion_yaw_tolerance", params_.inversion_yaw_tolerance);


  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &ParameterHandler::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ParameterHandler::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
}

ParameterHandler::~ParameterHandler()
{
  auto node = node_.lock();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();
}
rcl_interfaces::msg::SetParametersResult ParameterHandler::validateParameterUpdatesCallback(
  std::vector<rclcpp::Parameter> /*parameters*/)
{
  rcl_interfaces::msg::SetParametersResult result;
  
  return result;
}
void
ParameterHandler::updateParametersCallback(
  std::vector<rclcpp::Parameter> /*parameters*/)
{

}

}  // namespace nav2_controller
