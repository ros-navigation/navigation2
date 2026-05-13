// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#include "nav2_constrained_controller/logger.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace nav2_constrained_controller
{

std::string Logger::nowSuffix()
{
  auto t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  std::ostringstream ss;
  ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return ss.str();
}

static void mkdirIfMissing(const std::string & dir)
{
  struct stat st;
  if (stat(dir.c_str(), &st) != 0) {
    mkdir(dir.c_str(), 0755);
  }
}

void Logger::open(const std::string & log_dir, bool enabled)
{
  close();
  enabled_ = enabled;
  if (!enabled_) {return;}

  log_dir_ = log_dir;
  mkdirIfMissing(log_dir_);
  ts_suffix_ = nowSuffix();

  const auto path = [&](const std::string & stem) {
      return log_dir_ + "/" + stem + "_" + ts_suffix_ + ".csv";
    };

  f_main_.open(path("ctrl_main"), std::ios::trunc);
  f_path_.open(path("ctrl_path"), std::ios::trunc);
  f_walls_.open(path("ctrl_walls"), std::ios::trunc);
  f_cbf_.open(path("ctrl_cbf"), std::ios::trunc);
  f_qp_.open(path("ctrl_qp"), std::ios::trunc);
  f_lidar_.open(path("ctrl_lidar"), std::ios::trunc);
  f_centering_.open(path("ctrl_centering"), std::ios::trunc);
  f_events_.open(
    log_dir_ + "/ctrl_events_" + ts_suffix_ + ".txt", std::ios::trunc);

  writeHeaders();
  tick_ = 0;
}

void Logger::close()
{
  if (f_main_.is_open()) {f_main_.close();}
  if (f_path_.is_open()) {f_path_.close();}
  if (f_walls_.is_open()) {f_walls_.close();}
  if (f_cbf_.is_open()) {f_cbf_.close();}
  if (f_qp_.is_open()) {f_qp_.close();}
  if (f_lidar_.is_open()) {f_lidar_.close();}
  if (f_centering_.is_open()) {f_centering_.close();}
  if (f_events_.is_open()) {f_events_.close();}
}

void Logger::writeHeaders()
{
  f_main_ <<
    "tick,stamp_sec,rx,ry,ryaw,"
    "vx_nom,vy_nom,wz_nom,vx,vy,wz,reversing,dist_to_goal\n";
  f_path_ <<
    "tick,stamp_sec,xt,yt,yawt,r,s,ramp,yaw_err,sel_idx,n_poses\n";
  f_walls_ <<
    "tick,stamp_sec,id,lx,ly,c,p1x,p1y,p2x,p2y,length\n";
  f_cbf_ <<
    "tick,stamp_sec,kind,corner_id,wall_id,grad_x,grad_y,grad_w,"
    "rhs,h,u_nom_vx,u_nom_vy,u_nom_w,u_vx,u_vy,u_w,active\n";
  f_qp_ <<
    "tick,stamp_sec,ok,n_constraints,n_active,solve_time_us,"
    "deviation,iterations\n";
  f_lidar_ <<
    "tick,stamp_sec,idx,angle,range\n";
  f_centering_ <<
    "tick,stamp_sec,mode,needs_alignment,"
    "D_L,D_R,has_L,has_R,n_flanking,"
    "yaw_misalign,wall_quality,"
    "q_length,q_span,q_width,q_passage,"
    "passage_in_motion_dir,passage_distance,"
    "e_lat_passage,e_yaw_passage,alignment_error,"
    "vy_correction,wz_correction,vx_scale,"
    "vx_corrected,vy_corrected,wz_corrected\n";
  for (auto * f : {
      &f_main_, &f_path_, &f_walls_,
      &f_cbf_, &f_qp_, &f_lidar_, &f_centering_})
  {
    f->flush();
  }
}

uint64_t Logger::newTick(double /*stamp_sec*/)
{
  return ++tick_;
}

void Logger::event(const std::string & msg)
{
  if (!enabled_ || !f_events_.is_open()) {return;}
  auto t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  f_events_ << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
            << " | tick=" << tick_ << " | " << msg << "\n";
  f_events_.flush();
}

void Logger::logState(
  uint64_t tick, double stamp,
  double rx, double ry, double ryaw,
  const geometry_msgs::msg::Twist & u_nom,
  const geometry_msgs::msg::Twist & u_final,
  bool reversing, double dist_to_goal)
{
  if (!enabled_ || !f_main_.is_open()) {return;}
  f_main_ << std::fixed << std::setprecision(6)
          << tick << "," << stamp << ","
          << rx << "," << ry << "," << ryaw << ","
          << u_nom.linear.x << "," << u_nom.linear.y << "," << u_nom.angular.z << ","
          << u_final.linear.x << "," << u_final.linear.y << "," << u_final.angular.z << ","
          << (reversing ? 1 : 0) << "," << dist_to_goal << "\n";
  f_main_.flush();
}

void Logger::logPath(
  uint64_t tick, double stamp,
  double xt, double yt, double yawt,
  double r, double s, double ramp,
  double yaw_err, int sel_idx, int n_poses)
{
  if (!enabled_ || !f_path_.is_open()) {return;}
  f_path_ << std::fixed << std::setprecision(6)
          << tick << "," << stamp << ","
          << xt << "," << yt << "," << yawt << ","
          << r << "," << s << "," << ramp << "," << yaw_err << ","
          << sel_idx << "," << n_poses << "\n";
  f_path_.flush();
}

void Logger::logWalls(
  uint64_t tick, double stamp,
  const std::vector<Wall> & walls)
{
  if (!enabled_ || !f_walls_.is_open()) {return;}
  for (const auto & w : walls) {
    const double len = std::hypot(w.p2.x - w.p1.x, w.p2.y - w.p1.y);
    f_walls_ << std::fixed << std::setprecision(6)
             << tick << "," << stamp << "," << w.id << ","
             << w.lx << "," << w.ly << "," << w.c << ","
             << w.p1.x << "," << w.p1.y << ","
             << w.p2.x << "," << w.p2.y << "," << len << "\n";
  }
  f_walls_.flush();
}

void Logger::logCbfConstraints(
  uint64_t tick, double stamp,
  const std::vector<CbfConstraint> & cs,
  const Eigen::Vector3d & u_nom,
  const Eigen::Vector3d & u_final)
{
  if (!enabled_ || !f_cbf_.is_open()) {return;}
  for (const auto & c : cs) {
    // A CBF is "active" if h is small (close to the constraint
    // boundary). We log the raw value; downstream tooling can apply
    // its own threshold.
    const double lhs_at_u = c.grad.dot(u_final) + c.rhs;
    const bool active = lhs_at_u < 1e-3;
    f_cbf_ << std::fixed << std::setprecision(6)
           << tick << "," << stamp << ","
           << (c.kind == CbfConstraint::OUTER_WALL ? "outer" : "inner") << ","
           << c.corner_id << "," << c.wall_id << ","
           << c.grad.x() << "," << c.grad.y() << "," << c.grad.z() << ","
           << c.rhs << "," << c.h << ","
           << u_nom.x() << "," << u_nom.y() << "," << u_nom.z() << ","
           << u_final.x() << "," << u_final.y() << "," << u_final.z() << ","
           << (active ? 1 : 0) << "\n";
  }
  f_cbf_.flush();
}

void Logger::logQp(
  uint64_t tick, double stamp,
  bool ok, int n_constraints, int n_active,
  double solve_time_us, double deviation,
  int iterations)
{
  if (!enabled_ || !f_qp_.is_open()) {return;}
  f_qp_ << std::fixed << std::setprecision(6)
        << tick << "," << stamp << ","
        << (ok ? 1 : 0) << ","
        << n_constraints << "," << n_active << ","
        << solve_time_us << "," << deviation << "," << iterations << "\n";
  f_qp_.flush();
}

void Logger::logCentering(
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
  const geometry_msgs::msg::Twist & u_corrected)
{
  if (!enabled_ || !f_centering_.is_open()) {return;}
  f_centering_ << std::fixed << std::setprecision(6)
               << tick << "," << stamp << ","
               << mode << "," << (needs_alignment ? 1 : 0) << ","
               << D_L << "," << D_R << ","
               << (has_L ? 1 : 0) << "," << (has_R ? 1 : 0) << ","
               << n_flanking << ","
               << yaw_misalign << "," << wall_quality << ","
               << q_length << "," << q_span << ","
               << q_width << "," << q_passage << ","
               << (passage_in_motion_direction ? 1 : 0) << ","
               << passage_distance << ","
               << e_lat_passage << "," << e_yaw_passage << ","
               << alignment_error << ","
               << vy_correction << "," << wz_correction << ","
               << vx_scale << ","
               << u_corrected.linear.x << ","
               << u_corrected.linear.y << ","
               << u_corrected.angular.z << "\n";
  f_centering_.flush();
}

void Logger::logLidar(
  uint64_t tick, double stamp,
  const sensor_msgs::msg::LaserScan & scan)
{
  if (!enabled_ || !f_lidar_.is_open()) {return;}
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle = scan.angle_min + i * scan.angle_increment;
    f_lidar_ << std::fixed << std::setprecision(6)
             << tick << "," << stamp << ","
             << i << "," << angle << "," << scan.ranges[i] << "\n";
  }
  f_lidar_.flush();
}

}  // namespace nav2_constrained_controller
