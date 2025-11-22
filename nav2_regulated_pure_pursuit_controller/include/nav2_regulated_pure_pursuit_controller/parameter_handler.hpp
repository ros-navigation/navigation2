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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PARAMETER_HANDLER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/parameter_handler.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

struct Parameters
{
  double desired_linear_vel, base_desired_linear_vel;
  double lookahead_dist;
  double rotate_to_heading_angular_vel;
  double max_lookahead_dist;
  double min_lookahead_dist;
  double lookahead_time;
  bool use_velocity_scaled_lookahead_dist;
  double min_approach_linear_velocity;
  double approach_velocity_scaling_dist;
  double max_allowed_time_to_collision_up_to_carrot;
  double min_distance_to_obstacle;
  bool use_regulated_linear_velocity_scaling;
  bool use_cost_regulated_linear_velocity_scaling;
  double cost_scaling_dist;
  double cost_scaling_gain;
  double inflation_cost_scaling_factor;
  double regulated_linear_scaling_min_radius;
  double regulated_linear_scaling_min_speed;
  bool use_fixed_curvature_lookahead;
  double curvature_lookahead_dist;
  bool use_rotate_to_heading;
  double max_angular_accel;
  bool use_cancel_deceleration;
  double cancel_deceleration;
  double rotate_to_heading_min_angle;
  bool allow_reversing;
  double max_robot_pose_search_dist;
  bool interpolate_curvature_after_goal;
  bool use_collision_detection;
  double transform_tolerance;
  bool stateful;
};

/**
 * @class nav2_regulated_pure_pursuit_controller::ParameterHandler
 * @brief Handles parameters and dynamic parameters for RPP
 */
class ParameterHandler : public nav2_util::ParameterHandler<Parameters>
{
public:
  /**
   * @brief Constructor for nav2_regulated_pure_pursuit_controller::ParameterHandler
   */
  ParameterHandler(
    const nav2::LifecycleNode::SharedPtr & node,
    std::string & plugin_name,
    rclcpp::Logger & logger, const double costmap_size_x);

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

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PARAMETER_HANDLER_HPP_
