// Copyright 2024 Nav2 Contributors
// Licensed under the Apache License, Version 2.0

#ifndef EXAMPLE_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define EXAMPLE_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker.hpp"

namespace example_controller
{

/**
 * @class PurePursuitController
 * @brief Pure Pursuit path following controller for Nav2
 *
 * Implements the Pure Pursuit algorithm for smooth path following.
 * The controller selects a lookahead point on the path and computes
 * steering commands to drive toward it.
 */
class PurePursuitController : public nav2_core::Controller
{
public:
  /**
   * @brief Default constructor
   */
  PurePursuitController() = default;

  /**
   * @brief Virtual destructor
   */
  ~PurePursuitController() override = default;

  /**
   * @brief Configure the controller
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup resources
   */
  void cleanup() override;

  /**
   * @brief Activate the controller
   */
  void activate() override;

  /**
   * @brief Deactivate the controller
   */
  void deactivate() override;

  /**
   * @brief Set the path to follow
   */
  void setPath(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Compute velocity commands
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief Set speed limit
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief Reset controller state
   */
  void reset() override;

private:
  /**
   * @brief Transform global path to robot frame
   */
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Find lookahead point on the path
   */
  geometry_msgs::msg::PoseStamped getLookaheadPoint(
    const nav_msgs::msg::Path & transformed_plan,
    double lookahead_dist);

  /**
   * @brief Calculate curvature to reach lookahead point
   */
  double calculateCurvature(const geometry_msgs::msg::PoseStamped & lookahead);

  /**
   * @brief Check if path segment is collision-free
   */
  bool isCollisionImminent(
    const geometry_msgs::msg::PoseStamped & pose,
    double linear_vel, double angular_vel, double check_time);

  /**
   * @brief Publish visualization markers
   */
  void publishLookaheadMarker(const geometry_msgs::msg::PoseStamped & lookahead);

  // Node and infrastructure
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;

  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("PurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  // Frames
  std::string global_frame_;
  std::string robot_frame_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
    lookahead_pub_;

  // Path storage
  nav_msgs::msg::Path global_plan_;
  std::mutex path_mutex_;

  // Parameters
  double desired_linear_vel_;
  double max_linear_vel_;
  double min_linear_vel_;
  double max_angular_vel_;
  double lookahead_dist_;
  double min_lookahead_dist_;
  double max_lookahead_dist_;
  double lookahead_time_;
  bool use_velocity_scaled_lookahead_;
  double transform_tolerance_;
  double max_allowed_time_to_collision_;
  bool use_collision_detection_;
  bool use_rotate_to_heading_;
  double rotate_to_heading_angular_vel_;
  double rotate_to_heading_min_angle_;

  // Speed limit
  double speed_limit_;
  bool speed_limit_is_percentage_;
};

}  // namespace example_controller

#endif  // EXAMPLE_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
