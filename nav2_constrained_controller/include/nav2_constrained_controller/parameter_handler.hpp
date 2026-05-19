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

  // ---------- Stanley path-follower gains ----------
  double slowdown_radius{0.20};
  double k_yaw{1.5};
  double k_lat{0.8};

  // ---------- lookahead / path slicing ----------
  double motion_target_dist{0.30};
  double max_robot_pose_search_dist{2.0};
  double goal_dist_tolerance{0.05};

  // ---------- footprint rectangle (Saradagi Eq. 1) ----------
  // L is the full body length (front-to-back distance between the two
  // short edges), 2*db is the body width (rear-track lateral span).
  // dl, db are added safety margins on each axis. The CBF rectangle is
  // (L + 2*dl) x (2*db).
  // Robot physical dimensions: 900mm (length) x 750mm (width).
  double footprint_length{0.90};   // L  — full robot length
  double footprint_dl{0.05};       // dl — longitudinal margin
  double footprint_db{0.375};      // db — exact robot half-width (750mm / 2)

  // ---------- CBF / QP ----------
  double cbf_gamma{1.5};
  // Proximity speed scaling — disabled (set to 0). MPPI output passes through
  // at full speed; the predictive CBF QP is the correct speed controller.
  // Kept as a parameter so existing YAML files do not cause unknown-param errors.
  double wall_slow_h_thresh{0.0};
  // Predictive CBF: evaluate constraints at t+dt, t+2dt, ... using u_nom to
  // predict future perimeter positions. Catches Lie-derivative-degenerate cases
  // where the robot moves parallel to a wall that narrows ahead — the reactive
  // CBF sees L_g h ≈ 0 and does nothing, but the predicted positions reveal the
  // upcoming narrowing and generate tight constraints in the current tick.
  int    cbf_n_predict_steps{2};    // number of future steps to evaluate (0 = reactive only)
  double cbf_predict_dt{0.10};      // seconds per step (= 1 control tick at 10Hz)
  // Minimum safe distance (metres) subtracted from ESDF distance to form h.
  // h = d_esdf - d_safe. When h < 0 the robot has violated the safe margin.
  double esdf_d_safe{0.03};
  // CBF constraints with h > this value are skipped (robot far from obstacles,
  // constraint would be fully slack anyway).
  double wall_consideration_range{2.5};
  // Slack weight for soft CBF constraints. The QP solves:
  //   min ½||u-u_nom||² + (w/2)||ε||²  s.t. Au ≤ b+ε, ε≥0, box hard
  // Larger w → constraints harder (ε smaller). 0 → hard constraints (original).
  // At w=100: violating a constraint by 0.01 m/s costs 100×(0.01)²/2 = 0.005,
  // vs deviation cost of (0.01)²/2 = 0.00005 → slack is 100× more expensive.
  double cbf_slack_weight{100.0};

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
  double esdf_grid_resolution{0.01};  // metres per cell (1cm for ~50mm clearance)
  double esdf_grid_size_m{3.0};       // half-size (so 6m x 6m total)

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
