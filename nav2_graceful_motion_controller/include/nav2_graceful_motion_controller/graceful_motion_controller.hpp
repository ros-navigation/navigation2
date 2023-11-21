// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#ifndef NAV2_GRACEFUL_MOTION_CONTROLLER__GRACEFUL_MOTION_CONTROLLER_HPP_
#define NAV2_GRACEFUL_MOTION_CONTROLLER__GRACEFUL_MOTION_CONTROLLER_HPP_

#include <string>
#include <limits>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_graceful_motion_controller/path_handler.hpp"
#include "nav2_graceful_motion_controller/parameter_handler.hpp"
#include "nav2_graceful_motion_controller/smooth_control_law.hpp"

#include "visualization_msgs/msg/marker.hpp"

namespace nav2_graceful_motion_controller
{

/**
 * @class nav2_graceful_motion_controller::GracefulMotionController
 * @brief Graceful controller plugin
 */
class GracefulMotionController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for nav2_graceful_motion_controller::GracefulMotionController
   */
  GracefulMotionController() = default;

  /**
   * @brief Destructor for nav2_graceful_motion_controller::GracefulMotionController
   */
  ~GracefulMotionController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine.
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine.
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine.
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity.
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan.
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed
   * @param percentage setting speed limit in percentage if true
   * or in absolute values in false case
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Get motion target point.
   * @param motion_target_dist Optimal motion target distance
   * @param path Current global path
   * @return Motion target point
   */
  geometry_msgs::msg::PoseStamped getMotionTarget(
    const double & motion_target_dist,
    const nav_msgs::msg::Path & path);

  /**
   * @brief Create a PointStamped message of the motion target for
   * debugging / visualization porpuses.
   *
   * @param motion_target Motion target in PoseStamped format
   * @return geometry_msgs::msg::PointStamped Motion target in PointStamped format
   */
  geometry_msgs::msg::PointStamped createMotionTargetMsg(
    const geometry_msgs::msg::PoseStamped & motion_target);

  /**
   * @brief Create a flat circle marker of radius slowdown_radius around the motion target for
   * debugging / visualization porpuses.
   *
   * @param motion_target Motion target
   * @return visualization_msgs::msg::Marker Slowdown marker
   */
  visualization_msgs::msg::Marker createSlowdownMsg(
    const geometry_msgs::msg::PoseStamped & motion_target);

  /**
   * @brief Simulate trajectory calculating in every step the new velocity command based on
   * a new curvature value.
   *
   * @param motion_target Motion target point
   * @return nav_msgs::msg::Path Simulated trajectory
   */
  nav_msgs::msg::Path simulateTrajectory(
    const geometry_msgs::msg::PoseStamped & motion_target);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("GracefulMotionController")};

  Parameters * params_;
  double goal_dist_tolerance_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> transformed_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  motion_target_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>>
  slowdown_pub_;
  std::unique_ptr<nav2_graceful_motion_controller::PathHandler> path_handler_;
  std::unique_ptr<nav2_graceful_motion_controller::ParameterHandler> param_handler_;
  std::unique_ptr<nav2_graceful_motion_controller::SmoothControlLaw> control_law_;
};

}  // namespace nav2_graceful_motion_controller

#endif  // NAV2_GRACEFUL_MOTION_CONTROLLER__GRACEFUL_MOTION_CONTROLLER_HPP_
