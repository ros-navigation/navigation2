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

#ifndef NAV2_ROTATION_SHIM_CONTROLLER__PARAMETER_HANDLER_HPP_
#define NAV2_ROTATION_SHIM_CONTROLLER__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/parameter_handler.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace nav2_rotation_shim_controller
{

struct Parameters
{
  double forward_sampling_distance;
  double angular_dist_threshold;
  double angular_disengage_threshold;
  double rotate_to_heading_angular_vel;
  double max_angular_accel;
  double simulate_ahead_time;
  double control_duration;
  bool rotate_to_goal_heading;
  bool rotate_to_heading_once;
  bool closed_loop;
  bool use_path_orientations;
  std::string primary_controller;
};

/**
 * @class nav2_rotation_shim_controller::ParameterHandler
 * @brief Handles parameters and dynamic parameters for Rotation Shim
 */
class ParameterHandler : public nav2_util::ParameterHandler<Parameters>
{
public:
  /**
   * @brief Constructor for nav2_rotation_shim_controller::ParameterHandler
   */
  ParameterHandler(
    const nav2::LifecycleNode::SharedPtr & node,
    std::string & plugin_name,
    rclcpp::Logger & logger);

protected:
  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration of the node with the new parameter values.
   * @param parameters List of parameters that have been updated.
   */
  void
  updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters) override;

  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
   * @param parameters List of parameters that are being updated.
   * @return rcl_interfaces::msg::SetParametersResult Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult
  validateParameterUpdatesCallback(const std::vector<rclcpp::Parameter> & parameters) override;

  std::string plugin_name_;
};

}  // namespace nav2_rotation_shim_controller

#endif  // NAV2_ROTATION_SHIM_CONTROLLER__PARAMETER_HANDLER_HPP_
