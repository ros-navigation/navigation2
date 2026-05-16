// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONSTRAINED_CONTROLLER__PARAMETER_HANDLER_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__PARAMETER_HANDLER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_constrained_controller
{

// All controller parameters are kept in this struct. ParameterHandler
// owns the canonical instance; the rest of the plugin reads via
// getParams() and is expected to copy what it needs once per tick.
struct Parameters
{
  // ---------- nominal velocity envelope ----------
  double v_linear_min{0.05};
  double v_linear_max{0.4};
  double v_lateral_max{0.3};
  double v_angular_max{1.5};
  double v_linear_max_initial{0.4};
  double v_angular_max_initial{1.5};

  // ---------- nominal P-controller gains ----------
  double slowdown_radius{0.6};
  double k_yaw{1.5};
  bool   yaw_correction_ramp{true};

  // ---------- lookahead / path slicing ----------
  double motion_target_dist{0.4};
  double max_robot_pose_search_dist{2.0};
  // Goal tolerance below which we declare the segment finished.
  double goal_dist_tolerance{0.05};

  // ---------- footprint rectangle (Saradagi Eq. 1) ----------
  // L is the full body length (front-to-back distance between the two
  // short edges), 2*db is the body width (rear-track lateral span).
  // dl, db are added safety margins on each axis. The CBF rectangle is
  // (L + 2*dl) x (2*db).
  double footprint_length{0.65};   // L
  double footprint_dl{0.05};       // dl  (longitudinal margin)
  double footprint_db{0.30};       // db  (half-width incl. margin)

  // ---------- CBF / QP ----------
  double cbf_gamma{2.0};
  // Minimum safe distance (metres) subtracted from ESDF distance to form h.
  // h = d_esdf - d_safe. When h < 0 the robot has violated the safe margin.
  double esdf_d_safe{0.03};
  // CBF constraints with h > this value are skipped (robot far from obstacles,
  // constraint would be fully slack anyway).
  double wall_consideration_range{2.5};

  // ---------- PointCloud2 / ESDF ----------
  // Use horizontal beams from the raw 3D LiDAR.
  // z filter in base_link must be centred on the lidar mounting height
  // (1.347m) so only near-horizontal beams are used. These can see walls
  // at any lateral distance including 75mm door clearance (0° elevation).
  // The /scan approach uses lz=[0.162,0.312] which requires 65° elevation
  // for 75mm walls — beyond the Livox 52° max.
  std::string pointcloud_topic{"/livox/amr/lidar"};
  double esdf_z_min{1.247};   // lidar_height(1.347) - 0.1m
  double esdf_z_max{1.447};   // lidar_height(1.347) + 0.1m
  // ESDF grid parameters. Grid is square, centred on base_link origin.
  // Total grid extent = 2 * esdf_grid_size_m in each axis.
  double esdf_grid_resolution{0.04};  // metres per cell
  double esdf_grid_size_m{5.0};       // half-size (so 10m x 10m total)

  // ---------- logging ----------
  std::string log_dir{"/root/navigation_log"};
  bool   log_enabled{true};
  // Throttle for the (potentially large) raw lidar log: log every Nth
  // tick. Set to 0 to disable raw lidar logging entirely.
  int    log_lidar_every_n_ticks{10};
};

class ParameterHandler
{
public:
  ParameterHandler(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & plugin_name,
    rclcpp::Logger logger);

  ~ParameterHandler() = default;

  Parameters * getParams() {return &params_;}
  std::mutex & getMutex() {return mutex_;}

  // Dynamic reconfigure callback
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

protected:
  Parameters params_;
  std::mutex mutex_;
  std::string plugin_name_;
  rclcpp::Logger logger_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    dyn_params_handler_;
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__PARAMETER_HANDLER_HPP_
