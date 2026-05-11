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

#ifndef OPENNAV_DOCKING__PARAMETER_HANDLER_HPP_
#define OPENNAV_DOCKING__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/parameter_handler.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace opennav_docking
{

struct Parameters
{
  // Frequency to run control loops
  double controller_frequency;
  // Timeout for initially detecting the charge dock
  double initial_perception_timeout;
  // Timeout after making contact with dock for charging to start
  // If this is exceeded, the robot returns to the staging pose and retries
  double wait_charge_timeout;
  // Timeout to approach into the dock and reset its approach is retrying
  double dock_approach_timeout;
  // Timeout to rotate to the dock
  double rotate_to_dock_timeout;
  // When undocking, these are the tolerances for arriving at the staging pose
  double undock_linear_tolerance, undock_angular_tolerance;
  // Maximum number of times the robot will return to staging pose and retry docking
  int max_retries;
  // This is the root frame of the robot - typically "base_link"
  std::string base_frame;
  // This is our fixed frame for controlling - typically "odom"
  std::string fixed_frame;
  // The tolerance to the dock's staging pose not requiring navigation
  double dock_prestaging_tolerance;
  // Angular tolerance to exit the rotation loop when rotate_to_dock is enabled
  double rotation_angular_tolerance;
  // Does the robot drive backwards onto the dock? Default is forwards
  std::optional<bool> dock_backwards;
  // Parameters for OdomSmoother
  std::string odom_topic;
  double odom_duration;
};

/**
 * @class opennav_docking::ParameterHandler
 * @brief Handles parameters and dynamic parameters for Controller Server
 */
class ParameterHandler : public nav2_util::ParameterHandler<Parameters>
{
public:
  /**
   * @brief Constructor for opennav_docking::ParameterHandler
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
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__PARAMETER_HANDLER_HPP_
