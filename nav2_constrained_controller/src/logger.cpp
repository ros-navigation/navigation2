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
  if (stat(dir.c_str(), &st) != 0) {mkdir(dir.c_str(), 0755);}
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
  f_cbf_.open(path("ctrl_cbf"),   std::ios::trunc);
  f_qp_.open(path("ctrl_qp"),     std::ios::trunc);
  f_esdf3d_.open(path("ctrl_esdf3d"), std::ios::trunc);
  f_events_.open(
    log_dir_ + "/ctrl_events_" + ts_suffix_ + ".txt", std::ios::trunc);

  writeHeaders();
  tick_ = 0;
}

void Logger::close()
{
  if (f_main_.is_open())   {f_main_.close();}
  if (f_path_.is_open())   {f_path_.close();}
  if (f_cbf_.is_open())    {f_cbf_.close();}
  if (f_qp_.is_open())     {f_qp_.close();}
  if (f_esdf3d_.is_open()) {f_esdf3d_.close();}
  if (f_events_.is_open()) {f_events_.close();}
}

void Logger::writeHeaders()
{
  f_main_ <<
    "tick,stamp_sec,rx,ry,ryaw,"
    "vx_nom,vy_nom,wz_nom,vx,vy,wz,reversing,dist_to_goal,"
    "tx_odom_from_map,ty_odom_from_map,ryaw_odom_from_map\n";
  f_path_ <<
    "tick,stamp_sec,target_x,target_y,target_yaw,"
    "v_ref,dist_to_target,cross_track,yaw_err,target_idx,n_remaining,"
    "mode,v_scale\n";
  f_cbf_ <<
    "tick,stamp_sec,corner_id,grad_x,grad_y,grad_w,"
    "rhs,h,u_nom_vx,u_nom_vy,u_nom_w,u_vx,u_vy,u_w,active,"
    "sample_x,sample_y,predict_step\n";
  f_qp_ <<
    "tick,stamp_sec,ok,n_constraints,n_active,solve_time_us,"
    "deviation,iterations\n";
  f_esdf3d_ <<
    "tick,stamp_sec,n_near,n_below,n_in,n_above,min_z,max_z,"
    "nearest_oob_dist,nearest_oob_x,nearest_oob_y,nearest_oob_z\n";
  for (auto * f : {&f_main_, &f_path_, &f_cbf_, &f_qp_, &f_esdf3d_}) {
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
  bool reversing, double dist_to_goal,
  double tx_odom_from_map, double ty_odom_from_map, double ryaw_odom_from_map)
{
  if (!enabled_ || !f_main_.is_open()) {return;}
  f_main_ << std::fixed << std::setprecision(6)
          << tick << "," << stamp << ","
          << rx << "," << ry << "," << ryaw << ","
          << u_nom.linear.x << "," << u_nom.linear.y << "," << u_nom.angular.z << ","
          << u_final.linear.x << "," << u_final.linear.y << "," << u_final.angular.z << ","
          << (reversing ? 1 : 0) << "," << dist_to_goal << ","
          << tx_odom_from_map << "," << ty_odom_from_map << "," << ryaw_odom_from_map << "\n";
  f_main_.flush();
}

void Logger::logPath(
  uint64_t tick, double stamp,
  double target_x, double target_y, double target_yaw,
  double v_ref, double dist_to_target, double cross_track,
  double yaw_err, int target_idx, int n_remaining,
  int mode, double v_scale)
{
  if (!enabled_ || !f_path_.is_open()) {return;}
  f_path_ << std::fixed << std::setprecision(6)
          << tick << "," << stamp << ","
          << target_x << "," << target_y << "," << target_yaw << ","
          << v_ref << "," << dist_to_target << "," << cross_track << ","
          << yaw_err << "," << target_idx << "," << n_remaining << ","
          << mode << "," << v_scale << "\n";
  f_path_.flush();
}

void Logger::logCbfConstraints(
  uint64_t tick, double stamp,
  const std::vector<CbfConstraint> & cs,
  const Eigen::Vector3d & u_nom,
  const Eigen::Vector3d & u_final)
{
  if (!enabled_ || !f_cbf_.is_open()) {return;}
  for (const auto & c : cs) {
    const double lhs = c.grad.dot(u_final);
    const bool active = (lhs + c.rhs) < 1e-3;
    f_cbf_ << std::fixed << std::setprecision(6)
           << tick << "," << stamp << ","
           << c.corner_id << ","
           << c.grad.x() << "," << c.grad.y() << "," << c.grad.z() << ","
           << c.rhs << "," << c.h << ","
           << u_nom.x() << "," << u_nom.y() << "," << u_nom.z() << ","
           << u_final.x() << "," << u_final.y() << "," << u_final.z() << ","
           << (active ? 1 : 0) << ","
           << c.sample_x << "," << c.sample_y << "," << c.predict_step << "\n";
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

void Logger::logEsdf3d(
  uint64_t tick, double stamp,
  int n_near, int n_below, int n_in, int n_above,
  double min_z, double max_z,
  double nearest_oob_dist, double nearest_oob_x,
  double nearest_oob_y, double nearest_oob_z)
{
  if (!enabled_ || !f_esdf3d_.is_open()) {return;}
  f_esdf3d_ << std::fixed << std::setprecision(6)
            << tick << "," << stamp << ","
            << n_near << "," << n_below << "," << n_in << "," << n_above << ","
            << min_z << "," << max_z << ","
            << nearest_oob_dist << "," << nearest_oob_x << ","
            << nearest_oob_y << "," << nearest_oob_z << "\n";
  f_esdf3d_.flush();
}

}  // namespace nav2_constrained_controller
