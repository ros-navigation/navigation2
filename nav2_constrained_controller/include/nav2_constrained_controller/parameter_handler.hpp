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

  // ---------- LiDAR correction (NORMAL MODE) ----------
  // Path is primary; LiDAR adds small, clamped, deadbanded corrections to
  // vy and wz, gated by wall_quality. vx is untouched in normal mode.
  //
  // Proportional gain on lateral imbalance (D_L − D_R) → vy_correction.
  double k_lat{1.0};
  // Flanking wall test: a wall counts as a flanking corridor side iff its
  // segment tangent is approximately along robot +x. cos(20°) = 0.94.
  double flanking_cos_tol{0.94};
  // Hard cutoff on body-aware distance: a flanking-tangent wall farther
  // than this from the body is ignored (can't be the alley corridor wall).
  double max_centering_range{1.1};
  // Small additive clamps on the per-tick correction. These are tight on
  // purpose — corrections nudge, never override.
  double vy_correction_max{0.05};   // m/s
  double wz_correction_max{0.10};   // rad/s
  // Deadbands: no correction issued when error is within these tolerances.
  double centering_deadband_lat{0.02};   // m, |D_L − D_R| below this → no vy correction
  double centering_deadband_yaw{0.087};  // rad (~5°), |yaw_misalign| below this → no wz correction

  // ---------- Wall quality scoring ----------
  // wall_quality ∈ [0,1] gates the corrections in normal mode. It is the
  // product of several factors. Any factor near zero drops quality to zero.
  //
  // Minimum segment length to be considered a credible alley wall (m).
  double wall_quality_min_length{0.30};
  // Minimum fraction of body length the wall's x-range must span.
  double wall_quality_min_span_ratio{0.5};
  // Width consistency tolerance: |D_L + D_R + body_width − alley_width|
  // up to this much keeps quality high (m).
  double wall_quality_width_tol{0.20};
  // Expected alley width used for the width consistency factor (m).
  double wall_quality_expected_width{1.0};
  // Passage penalty: when a passage is detected within this distance of
  // the body (along motion direction), wall_quality drops toward zero.
  double passage_penalty_range{1.0};

  // ---------- Passage alignment mode (ALIGNMENT MODE) ----------
  // When a passage is detected in the motion direction within this range
  // AND the body is not yet aligned with the passage, the controller
  // switches to alignment mode: LiDAR drives vy and wz at full envelope
  // toward the gap center / passage axis; vx is scaled by alignment error.
  double alignment_passage_range{1.0};   // m
  // Alignment is considered achieved when both errors are below tolerance.
  double alignment_yaw_tol{0.087};       // rad (~5°)
  double alignment_lat_tol{0.03};        // m
  // Yaw and lateral error scale at which vx_scale reaches the floor.
  // alignment_error = max(|e_yaw|/yaw_scale, |e_lat|/lat_scale).
  // vx_scale = max(vx_align_floor, 1 − alignment_error).
  double alignment_yaw_scale{0.35};      // rad (~20°) → at this error, vx hits floor
  double alignment_lat_scale{0.10};      // m → at this error, vx hits floor
  // Minimum vx during alignment (fraction of u_path.vx). 0 = full stop allowed.
  double vx_align_floor{0.0};
  // Gain on alignment errors. With full envelope clamping these are aggressive.
  double k_lat_align{2.0};
  double k_yaw_align{2.0};

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
