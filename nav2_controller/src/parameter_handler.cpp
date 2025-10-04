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

  params_.controller_frequency = node->declare_or_get_parameter("controller_frequency", 20.0);
  params_.transform_tolerance = node->declare_or_get_parameter("transform_tolerance", 0.1);
  params_.min_x_velocity_threshold = node->declare_or_get_parameter("min_x_velocity_threshold", 0.0001);
  params_.min_y_velocity_threshold = node->declare_or_get_parameter("min_y_velocity_threshold", 0.0001);
  params_.min_theta_velocity_threshold = node->declare_or_get_parameter("min_theta_velocity_threshold", 0.0001);
  params_.speed_limit_topic = node->declare_or_get_parameter("speed_limit_topic", std::string("speed_limit"));
  params_.failure_tolerance = node->declare_or_get_parameter("failure_tolerance", 0.0);
  params_.use_realtime_priority = node->declare_or_get_parameter("use_realtime_priority", false);
  params_.publish_zero_velocity = node->declare_or_get_parameter("publish_zero_velocity", true);
  params_.costmap_update_timeout = node->declare_or_get_parameter("costmap_update_timeout", 0.30);
  params_.odom_topic = node->declare_or_get_parameter("odom_topic", std::string("odom"));
  params_.odom_duration = node->declare_or_get_parameter("odom_duration", 0.3);
  params_.interpolate_curvature_after_goal = node->declare_or_get_parameter("interpolate_curvature_after_goal", false);
  params_.max_robot_pose_search_dist = node->declare_or_get_parameter("max_robot_pose_search_dist", costmap_size_x / 2.0);
  params_.prune_distance = node->declare_or_get_parameter("prune_distance", 1.5);
  params_.enforce_path_inversion = node->declare_or_get_parameter("enforce_path_inversion", false);
  params_.inversion_xy_tolerance = node->declare_or_get_parameter("inversion_xy_tolerance", 0.2);
  params_.inversion_yaw_tolerance = node->declare_or_get_parameter("inversion_yaw_tolerance", 0.4);

  RCLCPP_INFO(
    logger_,
    "Controller frequency set to %.4fHz",
    params_.controller_frequency);

  if (params_.max_robot_pose_search_dist < 0.0) {
    RCLCPP_WARN(
      logger_, "Max robot search distance is negative, setting to max to search"
      " every point on path for the closest value.");
    params_.max_robot_pose_search_dist = std::numeric_limits<double>::max();
  }

  RCLCPP_INFO(logger_, "getting progress checker plugins..");
  params_.progress_checker_ids = node->declare_or_get_parameter("progress_checker_plugins", default_progress_checker_ids_);
  if (params_.progress_checker_ids == default_progress_checker_ids_) {
    for (size_t i = 0; i < default_progress_checker_ids_.size(); ++i) {
      nav2::declare_parameter_if_not_declared(
        node, default_progress_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_progress_checker_types_[i]));
    }
  }

  RCLCPP_INFO(logger_, "getting goal checker plugins..");
  params_.goal_checker_ids = node->declare_or_get_parameter("goal_checker_plugins", default_goal_checker_ids_);
  if (params_.goal_checker_ids == default_goal_checker_ids_) {
    for (size_t i = 0; i < default_goal_checker_ids_.size(); ++i) {
      nav2::declare_parameter_if_not_declared(
        node, default_goal_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_goal_checker_types_[i]));
    }
  }

  RCLCPP_INFO(logger_, "getting controller plugins..");
  params_.controller_ids = node->declare_or_get_parameter("controller_plugins", default_controller_ids_);
  if (params_.controller_ids == default_controller_ids_) {
    for (size_t i = 0; i < default_controller_ids_.size(); ++i) {
      nav2::declare_parameter_if_not_declared(
        node, default_controller_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_controller_types_[i]));
    }
  }

  params_.controller_types.resize(params_.controller_ids.size());
  params_.goal_checker_types.resize(params_.goal_checker_ids.size());
  params_.progress_checker_types.resize(params_.progress_checker_ids.size());

  for (size_t i = 0; i != params_.progress_checker_ids.size(); i++) {
    try {
      params_.progress_checker_types[i] = nav2::get_plugin_type_param(
        node, params_.progress_checker_ids[i]);
    } catch (const std::exception & ex) {
      throw std::runtime_error(
        std::string("Failed to get type for progress_checker '") +
        params_.progress_checker_ids[i] + "': " + ex.what());
    }
  }

  for (size_t i = 0; i != params_.goal_checker_ids.size(); i++) {
    try {
      params_.goal_checker_types[i] =
        nav2::get_plugin_type_param(node, params_.goal_checker_ids[i]);
    } catch (const std::exception & ex) {
      throw std::runtime_error(
        std::string("Failed to get type for goal_checker '") +
        params_.goal_checker_ids[i] + "': " + ex.what());
    }
  }

  for (size_t i = 0; i != params_.controller_ids.size(); i++) {
    try {
      params_.controller_types[i] = nav2::get_plugin_type_param(node, params_.controller_ids[i]);
    } catch (const std::exception & ex) {
      throw std::runtime_error(
        std::string("Failed to get type for controller plugins '") +
        params_.controller_types[i] + "': " + ex.what());
    }
  }

  // post_set_params_handler_ = node->add_post_set_parameters_callback(
  //   std::bind(
  //     &ParameterHandler::updateParametersCallback,
  //     this, std::placeholders::_1));
  // on_set_params_handler_ = node->add_on_set_parameters_callback(
  //   std::bind(
  //     &ParameterHandler::validateParameterUpdatesCallback,
  //     this, std::placeholders::_1));
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
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    // If we are trying to change the parameter of a plugin we can just skip it at this point
    // as they handle parameter changes themselves and don't need to lock the mutex
    if (param_name.find('.') != std::string::npos) {
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
ParameterHandler::updateParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name ==  "controller_frequency") {
        params_.controller_frequency = parameter.as_double();
      } else if (param_name == "transform_tolerance") {
        params_.transform_tolerance = parameter.as_double();
      } else if (param_name == "min_x_velocity_threshold") {
        params_.min_x_velocity_threshold = parameter.as_double();
      } else if (param_name == "min_y_velocity_threshold") {
        params_.min_y_velocity_threshold = parameter.as_double();
      } else if (param_name == "min_theta_velocity_threshold") {
        params_.min_theta_velocity_threshold = parameter.as_double();
      } else if (param_name == "failure_tolerance") {
        params_.failure_tolerance = parameter.as_double();
      } else if (param_name == "costmap_update_timeout") {
        params_.costmap_update_timeout = parameter.as_double();
      } else if (param_name == "odom_duration") {
        params_.odom_duration = parameter.as_double();
      } else if (param_name == "max_robot_pose_search_dist") {
        params_.max_robot_pose_search_dist = parameter.as_double();
      } else if (param_name == "prune_distance") {
        params_.prune_distance = parameter.as_double();
      } else if (param_name == "inversion_xy_tolerance") {
        params_.inversion_xy_tolerance = parameter.as_double();
      } else if (param_name == "inversion_yaw_tolerance") {
        params_.inversion_yaw_tolerance = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == "use_realtime_priority") {
        params_.use_realtime_priority = parameter.as_bool();
      } else if (param_name == "publish_zero_velocity") {
        params_.publish_zero_velocity = parameter.as_bool();
      } else if (param_name == "enforce_path_inversion") {
        params_.enforce_path_inversion = parameter.as_bool();
      }
    } else if (param_type == ParameterType::PARAMETER_STRING){
      if (param_name == "speed_limit_topic") {
        params_.speed_limit_topic = parameter.as_string();
      } else if (param_name == "odom_topic") {
        params_.odom_topic = parameter.as_string();
      }
    }
  }
}

}  // namespace nav2_controller
