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
#include "nav2_util/parameter_handler.hpp"
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
  rclcpp::Duration costmap_update_timeout{0, 0};
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
class ParameterHandler : public nav2_util::ParameterHandler<Parameters>
{
public:
  /**
   * @brief Constructor for nav2_controller::ParameterHandler
   */
  ParameterHandler(
    const nav2::LifecycleNode::SharedPtr & node,
    const rclcpp::Logger & logger);

protected:
  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
   * @param parameters List of parameters that are being updated.
   * @return rcl_interfaces::msg::SetParametersResult Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters) override;

  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration of the node with the new parameter values.
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters) override;

  std::string plugin_name_;
  const std::vector<std::string> default_progress_checker_ids_{"progress_checker"};
  const std::vector<std::string> default_progress_checker_types_{
    "nav2_controller::SimpleProgressChecker"};
  const std::vector<std::string> default_goal_checker_ids_{"goal_checker"};
  const std::vector<std::string> default_goal_checker_types_{"nav2_controller::SimpleGoalChecker"};
  const std::vector<std::string> default_controller_ids_{"FollowPath"};
  const std::vector<std::string> default_controller_types_{"nav2_mppi_controller::MPPIController"};
  const std::vector<std::string> default_path_handler_ids_{"PathHandler"};
  const std::vector<std::string> default_path_handler_types_{
    "nav2_controller::FeasiblePathHandler"};
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PARAMETER_HANDLER_HPP_
