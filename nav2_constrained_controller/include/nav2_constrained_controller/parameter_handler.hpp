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

  // ---------- PD path tracker ----------
  // u_nom comes from a PD tracker on the global lattice path (odom frame):
  //   1. Find the closest path pose to the robot (window-limited search,
  //      passed poses pruned).
  //   2. Walk lookahead_dist metres of arc-length ahead → target pose.
  //   3. vx,vy = R(-yaw) · (kp_pos · e_pos + kd_pos · de_pos/dt)   (odom-frame
  //      error, rotated into base_link), clamped per axis.
  //   4. wz = kp_yaw · yaw_err + kd_yaw · d(yaw_err)/dt where yaw_err is the
  //      shortest angular distance to the LATTICE's planned yaw at the target
  //      pose (OMNI motion model — path yaws carry intent, never retangented).
  // Body-aware safety is handled exclusively by the downstream CBF filter.
  double lookahead_dist{0.5};   // arc-length ahead of closest pose → target
  double kp_pos{1.0};           // P gain on odom-frame position error
  double kd_pos{0.0};           // D gain on d(position_err)/dt (odom frame —
                                // base-frame D would pick up rotation artifacts)
  double kp_yaw{0.8};           // P gain on lattice-yaw error
  double kd_yaw{0.0};           // D gain on d(yaw_err)/dt
  // k_yaw is used by the near-goal yaw-alignment branch in
  // computeVelocityCommands (the small-distance, snap-to-goal-heading mode).
  double k_yaw{1.5};

  // ---------- path slicing / goal tolerance ----------
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
  // Perimeter-sample spacing (m) for the CBF body polygon. Each rectangle side
  // gets ceil(side_len / spacing) + 1 samples (long sides incl. corners, short
  // sides interior-only). Smaller = finer polygon = more constraints. At 0.10
  // with the 1.00 x 0.75 m rectangle → 11 long + 9 short = 36 distinct points.
  // Dynamically reconfigurable for density sweeps.
  double cbf_sample_spacing_m{0.10};

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
