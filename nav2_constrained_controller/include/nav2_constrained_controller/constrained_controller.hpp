// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// ConstrainedController — Nav2 controller plugin for narrow-alley /
// door-to-door navigation on a holonomic AMR.
//
// Architecture (see ~/nav2_ws/AMR_CBF_Navigation_Design.pdf):
//
//   setPlan(path_in_map)
//     -> PathHandler::setPlan: one-shot AMCL touch, reproject into
//        odom, retangent constant-yaw lattice poses.
//
//   computeVelocityCommands(robot_pose):
//     1. PathHandler::getLocalPlan  -> base_link slice (no AMCL).
//     2. getMotionTarget            -> single lookahead pose (base_link).
//     3. SceneParser::parse         -> walls, corners, passage.
//     4. NominalController::compute -> u_nom (sign-symmetric P).
//     5. LateralCentering::compute  -> vy_walls, yaw_misalign.
//     6. Single-scalar blend (when both flanking walls visible):
//          vy = w*vy_walls + (1-w)*vy_path
//          wz = w*wz_walls + (1-w)*wz_path
//     7. CBFSafetyFilter::filter    -> u* = QP(u_blend, walls).
//     8. Logger::* per-tick rows.
//     9. Return TwistStamped(u*).

#ifndef NAV2_CONSTRAINED_CONTROLLER__CONSTRAINED_CONTROLLER_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__CONSTRAINED_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "nav2_constrained_controller/parameter_handler.hpp"
#include "nav2_constrained_controller/path_handler.hpp"
#include "nav2_constrained_controller/nominal_controller.hpp"
#include "nav2_constrained_controller/scene_parser.hpp"
#include "nav2_constrained_controller/lateral_centering.hpp"
#include "nav2_constrained_controller/cbf_safety_filter.hpp"
#include "nav2_constrained_controller/logger.hpp"

namespace nav2_constrained_controller
{

class ConstrainedController : public nav2_core::Controller
{
public:
  ConstrainedController() = default;
  ~ConstrainedController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
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
  // Lookahead picker — pure-pursuit by L2 distance, identical to
  // graceful's getMotionTarget, operating on the base_link slice
  // returned by PathHandler::getLocalPlan.
  geometry_msgs::msg::PoseStamped getMotionTarget(
    double motion_target_dist,
    const nav_msgs::msg::Path & local_plan,
    int * sel_idx_out) const;

  // LiDAR callback. Stores the most recent scan; the control tick
  // pulls from it.
  void lidarCallback(sensor_msgs::msg::LaserScan::SharedPtr msg);

  // Convert a stored LaserScan into points in base_link, applying
  // range filtering and the sensor->base_link TF.
  bool scanToBaseLink(
    const sensor_msgs::msg::LaserScan & scan,
    std::vector<Point2D> & out_points,
    const rclcpp::Time & stamp) const;

  // Reset to safe-zero command — used for goal-reached / fault paths.
  static geometry_msgs::msg::TwistStamped zeroTwist(
    const std::string & frame, const rclcpp::Time & stamp);

  // ROS plumbing.
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("ConstrainedController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Components (own all of them; constructed in configure()).
  std::unique_ptr<ParameterHandler> param_handler_;
  std::unique_ptr<PathHandler> path_handler_;
  std::unique_ptr<NominalController> nominal_;
  std::unique_ptr<SceneParser> scene_parser_;
  std::unique_ptr<LateralCentering> lateral_centering_;
  std::unique_ptr<CBFSafetyFilter> cbf_filter_;
  std::unique_ptr<Logger> log_;

  // LiDAR.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  // Visualisation publishers (best-effort; failures are non-fatal).
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    transformed_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::
    SharedPtr motion_target_pub_;

  // State.
  bool goal_reached_{false};
  uint64_t lidar_log_throttle_{0};
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__CONSTRAINED_CONTROLLER_HPP_
