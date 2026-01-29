// Copyright (c) 2025 Prabhav Saxena
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

#ifndef NAV2_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_controller
{

/**
 * @class PositionGoalChecker
 * @brief Goal Checker plugin that only checks XY position, ignoring orientation
 */
class PositionGoalChecker : public nav2_core::GoalChecker
{
public:
  /** 
   * @brief Construct a new Position Goal Checker object
   */
  PositionGoalChecker();

  /**
   * @brief Destroy the Position Goal Checker object
   */
  ~PositionGoalChecker() override = default;

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
   * @brief Registers callbacks for dynamic parameter handling.
   */
  void activate() override;

  /**
   * @brief Resets callbacks for dynamic parameter handling.
   */
  void deactivate() override;

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
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
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

  /**
   * @brief Set the XY goal tolerance
   * @param tolerance New tolerance value
   */
  void setXYGoalTolerance(double tolerance);

protected:
  nav2::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("position_goal_checker")};
  double xy_goal_tolerance_;
  double xy_goal_tolerance_sq_;
  double path_length_tolerance_;
  bool stateful_;
  bool position_reached_;
  std::string plugin_name_;
  std::mutex mutex_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;

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

#endif  // NAV2_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_
