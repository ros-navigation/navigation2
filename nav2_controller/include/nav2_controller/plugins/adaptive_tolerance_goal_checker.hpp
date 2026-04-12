// Copyright (c) 2026, David Grbac
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

#ifndef NAV2_CONTROLLER__PLUGINS__ADAPTIVE_TOLERANCE_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__ADAPTIVE_TOLERANCE_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace nav2_controller
{

/**
 * @class AdaptiveToleranceGoalChecker
 * @brief Goal Checker plugin with two tolerance tiers: a tight desired tolerance
 * and a looser coarse tolerance. The robot is considered to have reached the goal if:
 *   (1) it reaches within the desired (tight) tolerance, OR
 *   (2) it is within the coarse tolerance AND the robot's velocity is below
 *       a stopped threshold for a configurable number of consecutive cycles,
 *       indicating it is no longer making useful progress toward the goal.
 */
class AdaptiveToleranceGoalChecker : public nav2_core::GoalChecker
{
public:
  /**
   * @brief Construct a new Progress Goal Checker object
   */
  AdaptiveToleranceGoalChecker();
  /**
   * @brief Destroy the Progress Goal Checker object
   */
  ~AdaptiveToleranceGoalChecker();

  /**
   * @brief Initialize the goal checker
   * @param parent Weak pointer to the lifecycle node
   * @param plugin_name Name of the plugin
   * @param costmap_ros Shared pointer to the costmap
   */
  void initialize(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Reset the goal checker state
   */
  void reset() override;

  /**
   * @brief Check if the goal is reached
   * @param query_pose Current pose of the robot
   * @param goal_pose Target goal pose
   * @param velocity Current velocity of the robot
   * @param transformed_global_plan The transformed global plan
   * @return true if goal is reached, false otherwise
   */
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity,
    const nav_msgs::msg::Path & transformed_global_plan) override;

  /**
   * @brief Get the position and velocity tolerances
   * @param pose_tolerance Output parameter for pose tolerance
   * @param vel_tolerance Output parameter for velocity tolerance
   * @return true if tolerances are available, false otherwise
   */
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  nav2::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("adaptive_tolerance_goal_checker")};

  // Fine (desired) tolerance
  double fine_xy_goal_tolerance_;
  double fine_xy_goal_tolerance_sq_;
  // Coarse (fallback) tolerance
  double coarse_xy_goal_tolerance_;
  double coarse_xy_goal_tolerance_sq_;

  double yaw_goal_tolerance_;
  double path_length_tolerance_;
  bool stateful_;
  bool symmetric_yaw_tolerance_;

  // Velocity thresholds for detecting a stopped/stalled robot
  double trans_stopped_velocity_;
  double rot_stopped_velocity_;

  // Number of consecutive stopped cycles before accepting at coarse tolerance
  int required_stagnation_cycles_;

  // Stateful tracking
  bool check_xy_;
  bool in_tolerance_zone_;
  int stagnation_count_;
  double best_distance_sq_;

  // Dynamic parameters
  std::mutex mutex_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
  std::string plugin_name_;

  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
   * @param parameters List of parameters that are being updated.
   * @return rcl_interfaces::msg::SetParametersResult Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration of the node with the new parameter values.
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__ADAPTIVE_TOLERANCE_GOAL_CHECKER_HPP_
