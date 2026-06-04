// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// ConstrainedController — Nav2 controller plugin for constrained-space
// navigation (corridors, alleys, doorways, T-junctions) on a holonomic AMR.
//
// The global plan comes from Smac Lattice on the obstacle-layer-only costmap
// (planner_server getPlanOnlyInObstacles). Every pose carries a planned,
// footprint-validated yaw — the OMNI lattice's orientations encode intent
// (alley-axis hold, in-place rotations at doors) and are tracked verbatim.
//
// computeVelocityCommands() each tick:
//   1. ESDF update from LiDAR PointCloud2 (base_link, 1 cm).
//   2. dist_to_goal via live AMCL TF; near-goal yaw alignment branch returns
//      early once the position tolerance is reached.
//   3. PD path tracker on the odom-frame lattice path:
//        a. PathHandler::getTrackingTarget — closest pose (pruning passed
//           poses) + lookahead_dist arc-length walk → target pose.
//        b. vx,vy = R(-yaw) · PD(position error in odom), clamped per axis.
//        c. wz = PD(yaw error to the lattice's planned yaw at the target).
//   4. CBFSafetyFilter::filter → u* = slack QP(u_nom, 20-point perimeter +
//      predictive horizon).
//   5. Log + debug + stuck detection.
//
// The tracker treats the robot as a point; body-aware safety (rectangle
// corners on the live ESDF) is handled exclusively by the downstream CBF
// filter. Drift between BT replans is bounded by the CBF, which always acts
// on current sensor data in base_link.

#ifndef NAV2_CONSTRAINED_CONTROLLER__CONSTRAINED_CONTROLLER_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__CONSTRAINED_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_constrained_controller/parameter_handler.hpp"
#include "nav2_constrained_controller/path_handler.hpp"
#include "nav2_constrained_controller/esdf_grid.hpp"
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
  void pointcloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
  bool updateEsdf(const sensor_msgs::msg::PointCloud2 & cloud);

  // PD tracker on the odom-frame lattice path. Fills u_nom (body frame) and
  // the diagnostics outs (target pose in odom, yaw error, translation taper).
  // Yaw–translation coupling: the (vx, vy) vector is scaled by
  // max(0, cos(yaw_err)) so a misaligned body slows translation until wz
  // catches the heading up — prevents dragging the rectangle into walls
  // (run1 t300–500 failure) and enforces standing still during
  // rotate-in-place clusters. wz is never tapered. Returns false when no
  // target is available (caller should publish zero).
  bool trackPath(
    const geometry_msgs::msg::PoseStamped & robot_in_odom,
    const rclcpp::Time & stamp,
    geometry_msgs::msg::Twist & u_nom,
    PathHandler::TrackingTarget & target_out,
    double & yaw_err_out,
    double & v_scale_out);

  // Reset the PD differentiator memory (on new plan / activation).
  void resetTrackerState();

  static geometry_msgs::msg::TwistStamped zeroTwist(
    const std::string & frame, const rclcpp::Time & stamp);

  // ROS plumbing.
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("ConstrainedController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Components.
  std::unique_ptr<ParameterHandler> param_handler_;
  std::unique_ptr<PathHandler> path_handler_;
  std::unique_ptr<EsdfGrid> esdf_grid_;
  std::unique_ptr<CBFSafetyFilter> cbf_filter_;
  std::unique_ptr<Logger> log_;

  // LiDAR subscription and latest cloud (protected by cloud_mutex_).
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;

  // Cached static TF: base_link ← lidar frame.
  bool has_lidar_tf_{false};
  double tf_tx_{0.0}, tf_ty_{0.0}, tf_tz_{0.0};
  double tf_R00_{1.0}, tf_R01_{0.0}, tf_R02_{0.0};
  double tf_R10_{0.0}, tf_R11_{1.0}, tf_R12_{0.0};
  double tf_R20_{0.0}, tf_R21_{0.0}, tf_R22_{1.0};

  // Goal tracking (for dist_to_goal and near-goal yaw correction).
  geometry_msgs::msg::PoseStamped goal_in_map_;
  std::string odom_frame_{"odom"};
  bool has_plan_{false};

  // PD differentiator memory. Position error is kept in the ODOM frame —
  // base-frame error rates would pick up artifacts from the body rotating
  // under a stationary error. Reset on setPlan / activate.
  double prev_err_ox_{0.0};
  double prev_err_oy_{0.0};
  bool   have_prev_err_pos_{false};
  rclcpp::Time prev_pos_stamp_{0, 0, RCL_ROS_TIME};
  double prev_yaw_err_{0.0};
  bool   have_prev_yaw_err_{false};
  rclcpp::Time prev_yaw_stamp_{0, 0, RCL_ROS_TIME};
  // Last tick's TrackMode — used to emit one event per mode transition.
  int prev_track_mode_{0};

  // Visualisation publishers (live).
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr esdf_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr corners_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    velocity_arrow_pub_;
  // PD tracking target (published in base_link, transformed from odom).
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr
    tracking_target_pub_;
  // Scalar debug for rqt_plot (/constrained_controller/debug/data[0..11]):
  //   [0] h_min                 [1] qp_dev          [2] n_cst
  //   [3] vx_nom                [4] vy_nom          [5] wz_nom
  //   [6] vx*                   [7] vy*             [8] wz*
  //   [9] qp_infeasible(0/1)    [10] yaw_err        [11] dist_to_target
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>::SharedPtr
    cbf_constraints_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    cbf_gradients_pub_;

  bool goal_reached_{false};
  uint64_t esdf_pub_throttle_{0};
  double last_min_h_{1.0};
  bool stuck_{false};
  double stuck_start_x_{0.0};
  double stuck_start_y_{0.0};
  double stuck_start_yaw_{0.0};
  rclcpp::Time stuck_start_time_;
  // Near-goal yaw correction timeout.
  rclcpp::Time near_goal_yaw_start_;
  bool near_goal_yaw_active_{false};
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__CONSTRAINED_CONTROLLER_HPP_
