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
  // Width band for passage classifier (metres).
  double alley_width_min{0.95};
  double alley_width_max{1.10};
  double alley_width_tol{0.10};
  // Cutoff at which we consider a wall too far to bother emitting a CBF
  // for it. Guards against producing useless inactive constraints.
  double wall_consideration_range{2.5};
  // Inner-corner CBFs (Saradagi h5/h6) are gated off in the current
  // scope (straight alleys + door entry/exit only). The previous
  // implementation built a virtual wall along the passage line A–B,
  // which is wrong for an opening the robot must cross. Re-enable
  // only when L-bend support returns AND the emission rule is
  // rewritten to gate on real intersection corners, not passages.
  bool enable_inner_corner_cbf{false};

  // ---------- LiDAR scene parser ----------
  std::string lidar_topic{"/scan"};
  // Crop returns to this max range before fitting (metres). Returns
  // farther than this are treated as "open space" (no wall).
  double lidar_max_range{8.0};
  double lidar_min_range{0.05};
  // Split-and-merge thresholds.
  double line_split_threshold{0.04};   // metres, perpendicular distance
  int    line_min_points{8};
  // Two segments are "connected" iff their nearest endpoints are within
  // this distance. Otherwise they are disconnected (gap → CP rule).
  double segment_connect_dist{0.12};
  // Type-I parallelism tolerance (cosine of angle between normals).
  double parallel_cos_tol{0.95};
  // Type-II perpendicularity tolerance.
  double perp_cos_tol{0.20};

  // ---------- D_L/D_R LiDAR-based lateral centering ----------
  // Master switch. When true, vy_nom is overridden by the centering
  // law whenever at least one flanking wall is detected. Path-derived
  // vy is used as fallback (no flanking walls visible).
  bool   enable_lateral_centering{true};
  // Proportional gain on lateral error. vy = k_lat * (D_L - D_R).
  double k_lat{1.0};
  // A wall is "flanking" iff |t.x| >= flanking_cos_tol (its tangent is
  // roughly parallel to the robot's forward axis) AND its segment
  // x-range overlaps the robot body. cos(20°) = 0.94.
  double flanking_cos_tol{0.94};
  // Target half-width used by the single-wall (door-zone) regime.
  // vy = k_lat * (D_visible - target_half_width) when only one side
  // is observable. Defaults to mid of the alley_width band / 2.
  double target_half_width{0.5125};
  // Hard distance gate. A flanking wall farther than this (perpendicular
  // distance from robot origin to the wall LINE) is ignored — it cannot
  // be an alley wall. Without this, a stray distant wall in free space
  // can hijack the centering law and saturate vy. Default = 1.1 m
  // (wider than the worst-case 1.10 m alley).
  double max_centering_range{1.1};
  // If true, centering only fires when BOTH flanking walls are visible
  // and within max_centering_range. Single-side regimes are disabled.
  // Recommended: keep true. Single-side mode is dangerous in free
  // space where only one stray wall is visible.
  bool   require_both_walls{true};
  // Hysteresis: require this many consecutive ticks in the new regime
  // before switching from the previous one. Kills jitter at handoff.
  int    regime_switch_hyst_ticks{3};
  // Exponential low-pass on the centering vy command across ticks.
  // alpha = weight on the new sample (in [0,1]). 1 = no smoothing.
  double vy_centering_lpf_alpha{0.4};

  // ---------- Path-vs-walls blending (vy, wz) ----------
  // Single scalar weight applied to BOTH vy and wz when both flanking
  // walls are visible (walls_quality == 1, post-hysteresis):
  //   u = w * u_walls + (1 - w) * u_path
  // When walls_quality == 0 (one or no flanking walls — door / free
  // space), blend collapses to pure path. Default 1.0 = walls fully
  // dominate inside the alley, path takes over automatically at the
  // door zone via walls_quality.
  double wall_blend_weight{1.0};

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
