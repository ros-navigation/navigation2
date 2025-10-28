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

#ifndef NAV2_CONTROLLER__PARAMETER_HANDLER_HPP_
#define NAV2_CONTROLLER__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace nav2_controller
{

struct Parameters
{
  double controller_frequency;
  double min_x_velocity_threshold;
  double min_y_velocity_threshold;
  double min_theta_velocity_threshold;
  std::string speed_limit_topic;
  double failure_tolerance;
  bool use_realtime_priority;
  bool publish_zero_velocity;
  rclcpp::Duration  costmap_update_timeout{0,0};
  std::string odom_topic;
  double odom_duration;
  double search_window;
  std::vector<std::string> progress_checker_ids;
  std::vector<std::string> progress_checker_types;
  std::vector<std::string> goal_checker_ids;
  std::vector<std::string> goal_checker_types;
  std::vector<std::string> controller_ids;
  std::vector<std::string> controller_types;
  std::vector<std::string> path_handler_ids;
  std::vector<std::string> path_handler_types;
};

/**
 * @class nav2_controller::ParameterHandler
 * @brief Handles parameters and dynamic parameters for Controller Server
 */
class ParameterHandler
{
public:
  /**
   * @brief Constructor for nav2_controller::ParameterHandler
   */
  ParameterHandler(
    nav2::LifecycleNode::SharedPtr node,
    const rclcpp::Logger & logger);

  /**
   * @brief Destrructor for nav2_controller::ParameterHandler
   */
  ~ParameterHandler();

  std::mutex & getMutex() {return mutex_;}

  Parameters * getParams() {return &params_;}

  /**
  * @brief Registers callbacks for dynamic parameter handling.
  */
  void activate();

  /**
  * @brief Resets callbacks for dynamic parameter handling.
  */
  void deactivate();

protected:
  nav2::LifecycleNode::WeakPtr node_;

  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
   * @param parameters List of parameters that are being updated.
   * @return rcl_interfaces::msg::SetParametersResult Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration of the node with the new parameter values.
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
  Parameters params_;
  std::string plugin_name_;
  rclcpp::Logger logger_ {rclcpp::get_logger("ControllerServer")};

  const std::vector<std::string> default_progress_checker_ids_{"progress_checker"};
  const std::vector<std::string> default_progress_checker_types_{
    "nav2_controller::SimpleProgressChecker"};
  const std::vector<std::string> default_goal_checker_ids_{"goal_checker"};
  const std::vector<std::string> default_goal_checker_types_{"nav2_controller::SimpleGoalChecker"};
  const std::vector<std::string> default_controller_ids_{"FollowPath"};
  const std::vector<std::string> default_controller_types_{"dwb_core::DWBLocalPlanner"};
  const std::vector<std::string> default_path_handler_ids_{"path_handler"};
  const std::vector<std::string> default_path_handler_types_{"nav2_controller::SimplePathHandler"};
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PARAMETER_HANDLER_HPP_
