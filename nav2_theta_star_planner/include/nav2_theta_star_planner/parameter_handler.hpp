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

#ifndef NAV2_THETA_STAR_PLANNER__PARAMETER_HANDLER_HPP_
#define NAV2_THETA_STAR_PLANNER__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/parameter_handler.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace nav2_theta_star_planner
{

struct Parameters
{
  /// weight on the costmap traversal cost
  double w_traversal_cost;
  /// weight on the euclidean distance cost (used for calculations of g_cost)
  double w_euc_cost;
  /// weight on the heuristic cost (used for h_cost calculations)
  double w_heuristic_cost;
  /// parameter to set the number of adjacent nodes to be searched on
  int how_many_corners;
  /// parameter to set weather the planner can plan through unknown space
  bool allow_unknown;
  /// the interval at which the planner checks if it has been cancelled
  int terminal_checking_interval;
  bool use_final_approach_orientation;
};

/**
 * @class nav2_theta_star_planner::ParameterHandler
 * @brief Handles parameters and dynamic parameters for Theta Star
 */
class ParameterHandler : public nav2_util::ParameterHandler<Parameters>
{
public:
  /**
   * @brief Constructor for nav2_theta_star_planner::ParameterHandler
   */
  ParameterHandler(
    const nav2::LifecycleNode::SharedPtr & node,
    std::string & plugin_name,
    const rclcpp::Logger & logger);

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

}  // namespace nav2_theta_star_planner

#endif  // NAV2_THETA_STAR_PLANNER__PARAMETER_HANDLER_HPP_
