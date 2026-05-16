// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// Multi-stream CSV logger for the constrained-space controller.
// All files share tick_id so they can be joined offline.
//
//   ctrl_main_<ts>.csv  - one row per tick: state + nominal + final cmd
//   ctrl_path_<ts>.csv  - one row per tick: lookahead + path bookkeeping
//   ctrl_cbf_<ts>.csv   - one row per (tick, CBF constraint)
//   ctrl_qp_<ts>.csv    - one row per tick: QP solve metrics
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
    bool reversing, double dist_to_goal);

  void logPath(
    uint64_t tick, double stamp,
    double xt, double yt, double yawt,
    double r, double s, double ramp,
    double yaw_err, int sel_idx, int n_poses);

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

private:
  bool enabled_{false};
  uint64_t tick_{0};
  std::string log_dir_;
  std::string ts_suffix_;

  std::ofstream f_main_;
  std::ofstream f_path_;
  std::ofstream f_cbf_;
  std::ofstream f_qp_;
  std::ofstream f_events_;

  void writeHeaders();
  static std::string nowSuffix();
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__LOGGER_HPP_
