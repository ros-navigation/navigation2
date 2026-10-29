// Copyright (c) 2026 Pannaga Sudarshan
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

#ifndef NAV2_SMOOTHER__SMOOTHER_SERVER_PARAMETER_HANDLER_HPP_
#define NAV2_SMOOTHER__SMOOTHER_SERVER_PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/logger.hpp"
#include "rclcpp/parameter.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/parameter_handler.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace nav2_smoother
{

/**
 * @struct nav2_smoother::SmootherServerParameters
 * @brief Parameters for the SmootherServer
 */
struct SmootherServerParameters
{
  // Topic of the costmap used by the collision checker and passed to the plugins.
  // Not dynamically reconfigurable: plugins hold the subscriber created at configure time.
  std::string costmap_topic;
  // Topic of the robot footprint used by the collision checker
  std::string footprint_topic;
  // Frame the footprint is transformed into for collision checking
  std::string robot_base_frame;
  // Tolerance in seconds for footprint transform lookups
  double transform_tolerance;
  // Names of the smoother plugin instances to load.
  // Not dynamically reconfigurable: changing plugins requires a reconfigure.
  std::vector<std::string> smoother_plugins;
};

/**
 * @class nav2_smoother::SmootherServerParameterHandler
 * @brief Handles parameters and dynamic parameters for the SmootherServer
 */
class SmootherServerParameterHandler : public nav2_util::ParameterHandler<SmootherServerParameters>
{
public:
  /**
   * @brief Constructor for nav2_smoother::SmootherServerParameterHandler
   * @param node Lifecycle node that owns the parameters
   * @param logger Logger to use for warnings about rejected updates
   */
  SmootherServerParameterHandler(
    const nav2::LifecycleNode::SharedPtr & node,
    rclcpp::Logger & logger);

protected:
  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration with the new parameter values.
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters) override;

  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
   * @param parameters List of parameters that are being updated.
   * @return Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult
  validateParameterUpdatesCallback(const std::vector<rclcpp::Parameter> & parameters) override;
};

}  // namespace nav2_smoother

#endif  // NAV2_SMOOTHER__SMOOTHER_SERVER_PARAMETER_HANDLER_HPP_
