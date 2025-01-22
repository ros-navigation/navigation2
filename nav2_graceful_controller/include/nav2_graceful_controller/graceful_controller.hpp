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

#ifndef NAV2_GRACEFUL_CONTROLLER__GRACEFUL_CONTROLLER_HPP_
#define NAV2_GRACEFUL_CONTROLLER__GRACEFUL_CONTROLLER_HPP_

#include <string>
#include <limits>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_graceful_controller/path_handler.hpp"
#include "nav2_graceful_controller/parameter_handler.hpp"
#include "nav2_graceful_controller/smooth_control_law.hpp"
#include "nav2_graceful_controller/utils.hpp"

namespace nav2_graceful_controller
{

/**
 * @class nav2_graceful_controller::GracefulController
 * @brief Graceful controller plugin
 */
class GracefulController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for nav2_graceful_controller::GracefulController
   */
  GracefulController() = default;

  /**
   * @brief Destructor for nav2_graceful_controller::GracefulController
   */
  ~GracefulController() override = default;

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
   * @brief Simulate trajectory calculating in every step the new velocity command based on
   * a new curvature value and checking for collisions.
   *
   * @param motion_target Motion target point (in costmap local frame?)
   * @param costmap_transform Transform between global and local costmap
   * @param trajectory Simulated trajectory
   * @param cmd_vel Initial command velocity during simulation
   * @param backward Flag to indicate if the robot is moving backward
   * @return true if the trajectory is collision free, false otherwise
   */
  bool simulateTrajectory(
    const geometry_msgs::msg::PoseStamped & motion_target,
    const geometry_msgs::msg::TransformStamped & costmap_transform,
    nav_msgs::msg::Path & trajectory,
    geometry_msgs::msg::TwistStamped & cmd_vel,
    bool backward);

  /**
   * @brief Rotate the robot to face the motion target with maximum angular velocity.
   *
   * @param angle_to_target Angle to the motion target
   * @return geometry_msgs::msg::Twist Velocity command
   */
  geometry_msgs::msg::Twist rotateToTarget(double angle_to_target);

  /**
   * @brief Checks if the robot is in collision
   * @param x The x coordinate of the robot in global frame
   * @param y The y coordinate of the robot in global frame
   * @param theta The orientation of the robot in global frame
   * @return Whether in collision
   */
  bool inCollision(const double & x, const double & y, const double & theta);

  /**
   * @brief Compute the distance to each pose in a path
   * @param poses Poses to compute distances with
   * @param distances Computed distances
   */
  void computeDistanceAlongPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses,
    std::vector<double> & distances);

  /**
   * @brief Control law requires proper orientations, not all planners provide them
   * @param path Path to add orientations into, if required
   */
  void validateOrientations(std::vector<geometry_msgs::msg::PoseStamped> & path);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;
  rclcpp::Logger logger_{rclcpp::get_logger("GracefulController")};

  Parameters * params_;
  double goal_dist_tolerance_;
  bool goal_reached_;

  // True from the time a new path arrives until we have completed an initial rotation
  bool do_initial_rotation_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> transformed_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>>
  motion_target_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>>
  slowdown_pub_;
  std::unique_ptr<nav2_graceful_controller::PathHandler> path_handler_;
  std::unique_ptr<nav2_graceful_controller::ParameterHandler> param_handler_;
  std::unique_ptr<nav2_graceful_controller::SmoothControlLaw> control_law_;
};

}  // namespace nav2_graceful_controller

#endif  // NAV2_GRACEFUL_CONTROLLER__GRACEFUL_CONTROLLER_HPP_
