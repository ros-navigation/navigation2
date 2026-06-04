// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// Multi-stream CSV logger for the constrained-space controller.
// All files share tick_id so they can be joined offline.
//
//   ctrl_main_<ts>.csv   - one row per tick: state + nominal + final cmd
//   ctrl_path_<ts>.csv   - one row per tick: tracking target + path bookkeeping
//   ctrl_cbf_<ts>.csv    - one row per (tick, CBF constraint) + sample x/y/step
//   ctrl_qp_<ts>.csv     - one row per tick: QP solve metrics
//   ctrl_esdf3d_<ts>.csv - one row per tick: near-body LiDAR vertical profile
//                          (collision diagnostic — out-of-z-slice returns)
//   ctrl_events_<ts>.txt - free-form text: setPlan, errors, events

#ifndef NAV2_CONSTRAINED_CONTROLLER__LOGGER_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__LOGGER_HPP_

#include <fstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "Eigen/Core"

#include "nav2_constrained_controller/types.hpp"

namespace nav2_constrained_controller
{

class Logger
{
public:
  Logger() = default;
  ~Logger() {close();}

  void open(const std::string & log_dir, bool enabled);
  void close();

  bool isEnabled() const {return enabled_;}

  uint64_t newTick(double stamp_sec);
  void event(const std::string & msg);

  void logState(
    uint64_t tick, double stamp,
    double rx, double ry, double ryaw,
    const geometry_msgs::msg::Twist & u_nom,
    const geometry_msgs::msg::Twist & u_final,
    bool reversing, double dist_to_goal,
    // T_odom_from_map components — the per-tick AMCL transform. Pass NaN
    // if no plan / TF lookup failed. Lets us quantify AMCL drift relative
    // to setPlan instant in post-hoc analysis.
    double tx_odom_from_map, double ty_odom_from_map, double ryaw_odom_from_map);

  // One row per tick: PD tracker state.
  //   target_x/y/yaw  — lookahead target pose in odom (lattice yaw verbatim)
  //   v_ref           — |u_nom| linear magnitude before CBF (post-taper)
  //   dist_to_target  — euclidean robot → lookahead target
  //   cross_track     — euclidean robot → closest path pose (tracking quality)
  //   yaw_err         — shortest angular distance robot yaw → lattice yaw
  //   target_idx / n_remaining — path bookkeeping (pruned path)
  //   mode            — TrackMode: 0 NORMAL, 1 APPROACH_ROTATION,
  //                     2 ROTATE_IN_PLACE
  //   v_scale         — cos taper applied to (vx, vy); 1 = no taper
  void logPath(
    uint64_t tick, double stamp,
    double target_x, double target_y, double target_yaw,
    double v_ref, double dist_to_target, double cross_track,
    double yaw_err, int target_idx, int n_remaining,
    int mode, double v_scale);

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

  // One row per tick: near-body LiDAR vertical profile (collision diagnostic).
  // Probes raw cloud points inside the body footprint (+margin) in XY, bucketed
  // by height vs the ESDF z-slice [esdf_z_min, esdf_z_max]:
  //   n_near                 — # cloud points near the body in XY (any height)
  //   n_below / n_in / n_above — split by below-slice / in-slice / above-slice
  //   min_z / max_z          — vertical span of near-body returns
  //   nearest_oob_*          — nearest OUT-OF-SLICE point to the body: its
  //                            distance to the body edge + (x,y,z). dist≈0 with
  //                            an out-of-slice z = an obstacle the CBF is blind
  //                            to. n_near≈0 = lidar returns nothing near the body.
  void logEsdf3d(
    uint64_t tick, double stamp,
    int n_near, int n_below, int n_in, int n_above,
    double min_z, double max_z,
    double nearest_oob_dist, double nearest_oob_x,
    double nearest_oob_y, double nearest_oob_z);

private:
  bool enabled_{false};
  uint64_t tick_{0};
  std::string log_dir_;
  std::string ts_suffix_;

  std::ofstream f_main_;
  std::ofstream f_path_;
  std::ofstream f_cbf_;
  std::ofstream f_qp_;
  std::ofstream f_esdf3d_;
  std::ofstream f_events_;

  void writeHeaders();
  static std::string nowSuffix();
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__LOGGER_HPP_
