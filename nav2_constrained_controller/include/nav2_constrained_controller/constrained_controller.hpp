// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// ConstrainedController — Nav2 controller plugin for constrained-space
// navigation (corridors, alleys, doorways) on a holonomic AMR.
//
// computeVelocityCommands() each tick:
//   1. PathHandler::getLocalPlan  -> base_link slice (AMCL-free).
//   2. getMotionTarget            -> lookahead pose (base_link).
//   3. NominalController::compute -> u_nom (sign-symmetric P).
//   4. ESDF update from latest PointCloud2 (local, base_link frame).
//   5. CBFSafetyFilter::filter    -> u* = QP(u_nom, ESDF gradients).
//   6. Logger per-tick rows.
//   7. Return TwistStamped(u*).

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
#include "nav2_constrained_controller/nominal_controller.hpp"
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
  // Lookahead picker — pure-pursuit by L2 distance in base_link.
  geometry_msgs::msg::PoseStamped getMotionTarget(
    double motion_target_dist,
    const nav_msgs::msg::Path & local_plan,
    int * sel_idx_out) const;

  // PointCloud2 callback. Stores latest cloud for the control tick.
  void pointcloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Transform cloud to base_link, apply z filter (horizontal beams only),
  // project to 2D and rebuild ESDF. Returns false if TF lookup failed.
  bool updateEsdf(const sensor_msgs::msg::PointCloud2 & cloud);

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
  std::unique_ptr<NominalController> nominal_;
  std::unique_ptr<EsdfGrid> esdf_grid_;
  std::unique_ptr<CBFSafetyFilter> cbf_filter_;
  std::unique_ptr<Logger> log_;

  // PointCloud2 subscription and latest cloud (protected by cloud_mutex_).
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;

  // Cached static TF: base_link ← lidar frame.
  bool has_lidar_tf_{false};
  // Transform components cached for fast per-point application.
  double tf_tx_{0.0}, tf_ty_{0.0}, tf_tz_{0.0};
  double tf_R00_{1.0}, tf_R01_{0.0}, tf_R02_{0.0};
  double tf_R10_{0.0}, tf_R11_{1.0}, tf_R12_{0.0};
  double tf_R20_{0.0}, tf_R21_{0.0}, tf_R22_{1.0};

  // Visualisation publishers.
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    transformed_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr
    motion_target_pub_;
  // ESDF grid as OccupancyGrid (base_link frame, throttled at ~2Hz).
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    esdf_pub_;
  // Body corners as coloured spheres: green=safe, yellow=warning, red=tight.
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    corners_pub_;
  // Scalar debug metrics for rqt_plot:
  //   [0]  min_h across corners
  //   [1]  QP deviation ||u*-u_nom||
  //   [2]  n_constraints
  //   [3]  u_nom_vx   [4] u_nom_vy   [5] u_nom_wz
  //   [6]  u_star_vx  [7] u_star_vy  [8] u_star_wz
  //   [9]  fallback_fired (0/1)
  //   [10] wz_clamped (0/1) — wz suppressed due to narrow passage
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>::SharedPtr
    debug_pub_;

  bool goal_reached_{false};
  uint64_t cloud_log_throttle_{0};
  uint64_t esdf_pub_throttle_{0};
  double last_min_h_{1.0};  // min_h from previous tick — used for wz clamping
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__CONSTRAINED_CONTROLLER_HPP_
