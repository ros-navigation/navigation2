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
  GracefulController() = default;
  ~GracefulController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Legacy helper kept for test back-compat. Not used in the new control flow.
   */
  geometry_msgs::msg::PoseStamped getMotionTarget(
    const double & motion_target_dist,
    const nav_msgs::msg::Path & path);

  /**
   * @brief Simulate trajectory from origin in base frame toward motion_target, applying
   * an in-sim initial rotation (if enabled) then the smooth control law. cmd_vel is set
   * from the first simulation step. Returns false on collision.
   */
  bool simulateTrajectory(
    const geometry_msgs::msg::PoseStamped & motion_target,
    const geometry_msgs::msg::TransformStamped & costmap_transform,
    nav_msgs::msg::Path & trajectory,
    geometry_msgs::msg::TwistStamped & cmd_vel,
    bool backward);

  /**
   * @brief Rotate-in-place velocity command with v_angular_min_in_place floor.
   */
  geometry_msgs::msg::Twist rotateToTarget(const double & angle_to_target);

  bool inCollision(const double & x, const double & y, const double & theta);

  /**
   * @brief Precompute integrated path length from robot origin to each pose.
   */
  void computeDistanceAlongPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses,
    std::vector<double> & distances);

  /**
   * @brief If the planner emitted constant-yaw waypoints, retangent each pose so delta has meaning.
   */
  void validateOrientations(
    std::vector<geometry_msgs::msg::PoseStamped> & path);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;
  rclcpp::Logger logger_{rclcpp::get_logger("GracefulController")};

  Parameters * params_;
  double goal_dist_tolerance_;
  bool goal_reached_{false};
  bool do_initial_rotation_{false};

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> transformed_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  motion_target_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>>
  slowdown_pub_;
  std::unique_ptr<nav2_graceful_controller::PathHandler> path_handler_;
  std::unique_ptr<nav2_graceful_controller::ParameterHandler> param_handler_;
  std::unique_ptr<nav2_graceful_controller::SmoothControlLaw> control_law_;
};

}  // namespace nav2_graceful_controller

#endif  // NAV2_GRACEFUL_CONTROLLER__GRACEFUL_CONTROLLER_HPP_
