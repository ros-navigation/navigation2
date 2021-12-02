// Copyright (c) 2020 Samsung Research America
// Copyright (c) 2021 Alex Melkobrodov
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

#ifndef NAV2_SHIM_CONTROLLER__SHIM_CONTROLLER_HPP_
#define NAV2_SHIM_CONTROLLER__SHIM_CONTROLLER_HPP_

#include <string>
#include <memory>
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/exceptions.hpp"
#include "angles/angles.h"

namespace nav2_shim_controller
{

/**
 * @class nav2_shim_controller::ShimController
 * @brief Shim controller plugin
 */
class ShimController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for nav2_shim_controller::ShimController
   */
  ShimController() = default;

  /**
   * @brief Destrructor for nav2_shim_controller::ShimController
   */
  ~ShimController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Controller computeVelocityCommands - calculates the best command given the current pose and velocity
   *
   * It is presumed that the global plan is already set.
   *
   * This is mostly a wrapper for the protected computeVelocityCommands
   * function which has additional debugging info.
   *
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @param goal_checker Pointer to the current goal checker the task is utilizing
   * @return The best command for the robot to drive
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Whether robot should rotate to rough path heading
   * @param robot_pose current robot pose
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @param angle_thresh Angle threshold
   * @return Whether should rotate to path heading
   */
  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & robot_pose, double & angle_to_path,
    const double & angle_thresh);

  /**
   * @brief Whether robot should rotate to final goal orientation
   * @param robot_pose current robot pose
   * @param carrot_pose current lookahead point
   * @return Whether should rotate to goal heading
   */
  bool isCloseToGoalHeading(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Create a smooth and kinematically smoothed rotation command
   * @param angular_vel angular velocity
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @param curr_speed the current robot speed
   */
  void rotateToHeading(
    double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::TwistStamped & curr_speed);

  /**
   * @brief Transform a pose to another frame.
   * @param input_pose input pose which need to transform
   * @return pose in base frame
   */
  geometry_msgs::msg::PoseStamped transformPoseToMapFrame(
    const geometry_msgs::msg::PoseStamped & input_pose);

  std::string plugin_name_;
  rclcpp::Logger logger_ {rclcpp::get_logger("ShimController")};
  std::shared_ptr<tf2_ros::Buffer> tf_;

  double max_angular_vel_;
  double max_linear_vel_;
  tf2::Duration transform_tolerance_;
  double control_duration_;
  double max_angular_accel_;
  double goal_dist_tol_;
  double goal_yaw_tol_;
  double max_angle_threshold_;
  bool use_rotate_to_heading_, use_rotate_to_path_;
  bool use_dynamic_threshold_;
  std::string default_plugin_name_;
  geometry_msgs::msg::TwistStamped prev_cmd_vel_;

  nav_msgs::msg::Path global_plan_;

  pluginlib::ClassLoader<Controller> plugin_loader_{"nav2_core", "nav2_core::Controller"};
  std::shared_ptr<Controller> default_plugin;
};

}  // namespace nav2_shim_controller

#endif  // NAV2_SHIM_CONTROLLER__SHIM_CONTROLLER_HPP_
