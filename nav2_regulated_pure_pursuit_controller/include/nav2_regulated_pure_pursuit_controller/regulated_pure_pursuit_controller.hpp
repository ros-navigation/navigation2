// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_regulated_pure_pursuit_controller/path_handler.hpp"
#include "nav2_regulated_pure_pursuit_controller/collision_checker.hpp"
#include "nav2_regulated_pure_pursuit_controller/parameter_handler.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulation_functions.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

/**
 * @class nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
 * @brief Regulated pure pursuit controller plugin
 */
class RegulatedPurePursuitController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
   */
  RegulatedPurePursuitController() = default;

  /**
   * @brief Destrructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
   */
  ~RegulatedPurePursuitController() override = default;

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
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  bool cancel() override;

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

  void reset() override;

protected:
  /**
   * @brief Get lookahead distance
   * @param cmd the current speed to use to compute lookahead point
   * @return lookahead distance
   */
  double getLookAheadDistance(const geometry_msgs::msg::Twist &);

  /**
   * @brief Creates a PointStamped message for visualization
   * @param carrot_pose Input carrot point as a PoseStamped
   * @return CarrotMsg a carrot point marker, PointStamped
   */
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Whether robot should rotate to rough path heading
   * @param carrot_pose current lookahead point
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @param x_vel_sign Velocoty sign (forward or backward)
   * @return Whether should rotate to path heading
   */
  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path,
    double & x_vel_sign);

  /**
   * @brief Whether robot should rotate to final goal orientation
   * @param carrot_pose current lookahead point
   * @return Whether should rotate to goal heading
   */
  bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Create a smooth and kinematically smoothed rotation command
   * @param linear_vel linear velocity
   * @param angular_vel angular velocity
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @param curr_speed the current robot speed
   */
  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed);

  /**
   * @brief apply regulation constraints to the system
   * @param linear_vel robot command linear velocity input
   * @param lookahead_dist optimal lookahead distance
   * @param curvature curvature of path
   * @param speed Speed of robot
   * @param pose_cost cost at this pose
   */
  void applyConstraints(
    const double & curvature, const geometry_msgs::msg::Twist & speed,
    const double & pose_cost, const nav_msgs::msg::Path & path,
    double & linear_vel, double & sign);

  /**
   * @brief Find the intersection a circle and a line segment.
   * This assumes the circle is centered at the origin.
   * If no intersection is found, a floating point error will occur.
   * @param p1 first endpoint of line segment
   * @param p2 second endpoint of line segment
   * @param r radius of circle
   * @return point of intersection
   */
  static geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    double r);

  /**
   * @brief Get lookahead point
   * @param lookahead_dist Optimal lookahead distance
   * @param path Current global path
   * @param interpolate_after_goal If true, interpolate the lookahead point after the goal based
   * on the orientation given by the position of the last two pose of the path
   * @return Lookahead point
   */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    const double &, const nav_msgs::msg::Path &,
    bool interpolate_after_goal = false);

  /**
   * @brief checks for the cusp position
   * @param pose Pose input to determine the cusp position
   * @return robot distance from the cusp
   */
  double findVelocitySignChange(const nav_msgs::msg::Path & transformed_plan);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("RegulatedPurePursuitController")};

  Parameters * params_;
  double goal_dist_tol_;
  double control_duration_;
  bool cancelling_ = false;
  bool finished_cancelling_ = false;
  bool is_rotating_to_heading_ = false;
  bool has_reached_xy_tolerance_ = false;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  carrot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  curvature_carrot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>>
  is_rotating_to_heading_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> carrot_arc_pub_;
  std::unique_ptr<nav2_regulated_pure_pursuit_controller::PathHandler> path_handler_;
  std::unique_ptr<nav2_regulated_pure_pursuit_controller::ParameterHandler> param_handler_;
  std::unique_ptr<nav2_regulated_pure_pursuit_controller::CollisionChecker> collision_checker_;
};

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
