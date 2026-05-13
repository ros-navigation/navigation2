// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// Multi-stream CSV logger for the constrained-space controller.
//
// We split logs across several files because a single wide row would
// require dummy fields for variable-length structures (per-tick wall
// list, per-tick CBF constraint set, etc). All files share the same
// `tick_id` column so they can be joined offline.
//
// Files (timestamped at activation, written under params.log_dir):
//   ctrl_main_<ts>.csv      - one row per tick: state + nominal + final
//   ctrl_path_<ts>.csv      - one row per tick: lookahead + path bookkeeping
//   ctrl_walls_<ts>.csv     - one row per (tick, wall)
//   ctrl_corners_<ts>.csv   - one row per (tick, corner)
//   ctrl_passage_<ts>.csv   - one row per tick: detected passage state
//   ctrl_cbf_<ts>.csv       - one row per (tick, cbf constraint)
//   ctrl_qp_<ts>.csv        - one row per tick: QP solve metrics
//   ctrl_lidar_<ts>.csv     - one row per (tick, return) - throttled
//   ctrl_centering_<ts>.csv - one row per tick: lateral centering + blend
//   ctrl_events_<ts>.txt    - free-form text: setPlan, errors, regime changes
//
// Logger is intentionally synchronous and unbuffered-on-flush so that
// crashes preserve the last lines. This is fine for the 20-50 Hz
// control rate; revisit only if profiling says so.

#ifndef NAV2_CONSTRAINED_CONTROLLER__LOGGER_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__LOGGER_HPP_

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_constrained_controller/types.hpp"

namespace nav2_constrained_controller
{

class Logger
{
public:
  Logger() = default;
  ~Logger() {close();}

  // Open all log streams. Safe to call repeatedly — each call rotates
  // the timestamp suffix and reopens fresh files.
  void open(const std::string & log_dir, bool enabled);
  void close();

  bool isEnabled() const {return enabled_;}

  // Bump the global tick counter and stamp. Returns the new tick id so
  // the caller can pass it through to per-snapshot logging methods.
  uint64_t newTick(double stamp_sec);

  // Free-form event line. Used for setPlan, errors, regime changes.
  void event(const std::string & msg);

  // Per-tick logging methods. All take the current tick id explicitly
  // so the caller can emit rows in any order.
  void logState(
    uint64_t tick, double stamp,
    double rx, double ry, double ryaw,
    const geometry_msgs::msg::Twist & u_nom,
    const geometry_msgs::msg::Twist & u_final,
    bool reversing, double dist_to_goal);

  void logPath(
    uint64_t tick, double stamp,
    double xt, double yt, double yawt,
    double r, double s, double ramp,
    double yaw_err, int sel_idx, int n_poses);

  void logWalls(
    uint64_t tick, double stamp,
    const std::vector<Wall> & walls);

  void logCbfConstraints(
    uint64_t tick, double stamp,
    const std::vector<CbfConstraint> & cs,
    const Eigen::Vector3d & u_nom,
    const Eigen::Vector3d & u_final);

  void logQp(
    uint64_t tick, double stamp,
    bool ok, int n_constraints, int n_active,
    double solve_time_us, double deviation,
    int iterations);

  // Throttled by params.log_lidar_every_n_ticks (handled at call site).
  void logLidar(
    uint64_t tick, double stamp,
    const sensor_msgs::msg::LaserScan & scan);

  // Lateral-centering snapshot. mode: 0=NORMAL, 1=ALIGNMENT.
  // D_L/D_R are body-aware Voronoi-region segment-distances to the
  // closest flanking wall on each side. yaw_misalign is the angle
  // (rad) between the alley axis (avg flanking-wall tangent, +x-
  // folded) and robot +x. wall_quality is the composite [0,1] gate
  // used by NORMAL mode; q_length/q_span/q_width/q_passage are its
  // factors (q_count is redundant with has_L && has_R). Passage and
  // alignment fields come from the passage classifier;
  // alignment_error = max(|e_yaw|/yaw_scale, |e_lat|/lat_scale).
  // vy_correction, wz_correction, vx_scale capture what the
  // controller actually applied this tick. u_corrected is the
  // post-correction, pre-CBF twist that the CBF saw as nominal.
  void logCentering(
    uint64_t tick, double stamp,
    int mode, bool needs_alignment,
    double D_L, double D_R,
    bool has_L, bool has_R, int n_flanking,
    double yaw_misalign, double wall_quality,
    double q_length, double q_span, double q_width, double q_passage,
    bool passage_in_motion_direction,
    double passage_distance,
    double e_lat_passage, double e_yaw_passage, double alignment_error,
    double vy_correction, double wz_correction, double vx_scale,
    const geometry_msgs::msg::Twist & u_corrected);

private:
  bool enabled_{false};
  uint64_t tick_{0};
  std::string log_dir_;
  std::string ts_suffix_;

  std::ofstream f_main_;
  std::ofstream f_path_;
  std::ofstream f_walls_;
  std::ofstream f_cbf_;
  std::ofstream f_qp_;
  std::ofstream f_lidar_;
  std::ofstream f_events_;
  std::ofstream f_centering_;

  void writeHeaders();
  static std::string nowSuffix();
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__LOGGER_HPP_
