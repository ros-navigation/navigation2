// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#include "nav2_constrained_controller/constrained_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/utils.h"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace nav2_constrained_controller
{

void ConstrainedController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  auto node = parent.lock();
  if (!node) {throw std::runtime_error("ConstrainedController: invalid parent node");}

  plugin_name_  = name;
  tf_buffer_    = tf;
  costmap_ros_  = costmap_ros;
  logger_       = node->get_logger();
  clock_        = node->get_clock();

  param_handler_ = std::make_unique<ParameterHandler>(parent, plugin_name_, logger_);
  auto * params  = param_handler_->getParams();

  const tf2::Duration transform_tol = tf2::durationFromSec(0.1);
  path_handler_ = std::make_unique<PathHandler>(transform_tol, tf_buffer_, costmap_ros_);
  nominal_      = std::make_unique<NominalController>(params);
  esdf_grid_    = std::make_unique<EsdfGrid>(params);
  cbf_filter_   = std::make_unique<CBFSafetyFilter>(params);
  log_          = std::make_unique<Logger>();

  transformed_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "constrained_controller/transformed_local_plan", 1);
  motion_target_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "constrained_controller/motion_target", 1);
  esdf_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "constrained_controller/esdf_grid", rclcpp::SystemDefaultsQoS());
  corners_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "constrained_controller/body_corners", rclcpp::SystemDefaultsQoS());
  debug_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>(
    "constrained_controller/debug", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    logger_,
    "ConstrainedController '%s' configured. log_dir=%s, cloud_topic=%s",
    plugin_name_.c_str(),
    params->log_dir.c_str(),
    params->pointcloud_topic.c_str());
}

void ConstrainedController::activate()
{
  auto node = parent_.lock();
  if (!node) {return;}
  auto * params = param_handler_->getParams();

  log_->open(params->log_dir, params->log_enabled);
  log_->event("activate");

  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    params->pointcloud_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&ConstrainedController::pointcloudCallback, this, std::placeholders::_1));

  transformed_plan_pub_->on_activate();
  motion_target_pub_->on_activate();
  esdf_pub_->on_activate();
  corners_pub_->on_activate();
  debug_pub_->on_activate();

  goal_reached_       = false;
  cloud_log_throttle_ = 0;
  esdf_pub_throttle_  = 0;
  has_lidar_tf_       = false;
  last_min_h_         = 1.0;
  stuck_ticks_        = 0;
  stuck_              = false;

  RCLCPP_INFO(logger_, "ConstrainedController activated, subscribing to %s",
    params->pointcloud_topic.c_str());
}

void ConstrainedController::deactivate()
{
  if (log_) {log_->event("deactivate"); log_->close();}
  cloud_sub_.reset();
  if (transformed_plan_pub_) {transformed_plan_pub_->on_deactivate();}
  if (motion_target_pub_)    {motion_target_pub_->on_deactivate();}
  if (esdf_pub_)             {esdf_pub_->on_deactivate();}
  if (corners_pub_)          {corners_pub_->on_deactivate();}
  if (debug_pub_)            {debug_pub_->on_deactivate();}
}

void ConstrainedController::cleanup()
{
  param_handler_.reset();
  path_handler_.reset();
  nominal_.reset();
  esdf_grid_.reset();
  cbf_filter_.reset();
  log_.reset();
  transformed_plan_pub_.reset();
  motion_target_pub_.reset();
}

void ConstrainedController::setPlan(const nav_msgs::msg::Path & path)
{
  const bool ok = path_handler_->setPlan(path);
  goal_reached_ = false;
  stuck_        = false;
  stuck_ticks_  = 0;
  esdf_grid_->reset();
  if (log_) {
    log_->event(
      std::string("setPlan ") + (ok ? "ok" : "FAILED") +
      " | poses=" + std::to_string(path.poses.size()) +
      " | frame=" + path.header.frame_id);
  }
}

void ConstrainedController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(param_handler_->getMutex());
  auto * p = param_handler_->getParams();
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    p->v_linear_max = p->v_linear_max_initial;
    p->v_angular_max = p->v_angular_max_initial;
    return;
  }
  if (percentage) {
    p->v_linear_max  = std::max(p->v_linear_max_initial * speed_limit / 100.0, p->v_linear_min);
    p->v_angular_max = p->v_angular_max_initial * speed_limit / 100.0;
  } else {
    p->v_linear_max  = std::max(speed_limit, p->v_linear_min);
    p->v_angular_max = p->v_angular_max_initial *
      speed_limit / std::max(1e-6, p->v_linear_max_initial);
  }
}

void ConstrainedController::pointcloudCallback(
  sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  last_cloud_ = msg;
}

bool ConstrainedController::updateEsdf(const sensor_msgs::msg::PointCloud2 & cloud)
{
  const std::string base_frame = costmap_ros_->getBaseFrameID();

  // Cache the static lidar→base_link transform after first lookup.
  if (!has_lidar_tf_) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        base_frame, cloud.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "updateEsdf: TF %s<-%s failed: %s",
        base_frame.c_str(), cloud.header.frame_id.c_str(), e.what());
      return false;
    }
    const auto & t = tf_stamped.transform.translation;
    const auto & r = tf_stamped.transform.rotation;
    tf_tx_ = t.x; tf_ty_ = t.y; tf_tz_ = t.z;
    const double qx=r.x, qy=r.y, qz=r.z, qw=r.w;
    tf_R00_ = 1-2*(qy*qy+qz*qz); tf_R01_ = 2*(qx*qy-qw*qz); tf_R02_ = 2*(qx*qz+qw*qy);
    tf_R10_ = 2*(qx*qy+qw*qz);   tf_R11_ = 1-2*(qx*qx+qz*qz); tf_R12_ = 2*(qy*qz-qw*qx);
    tf_R20_ = 2*(qx*qz-qw*qy);   tf_R21_ = 2*(qy*qz+qw*qx);   tf_R22_ = 1-2*(qx*qx+qy*qy);
    has_lidar_tf_ = true;
  }

  const auto * params = param_handler_->getParams();
  const double z_min = params->esdf_z_min;
  const double z_max = params->esdf_z_max;

  std::vector<Point2D> pts;
  pts.reserve(cloud.width * cloud.height);

  sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iy(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iz(cloud, "z");

  for (; ix != ix.end(); ++ix, ++iy, ++iz) {
    const float px = *ix, py = *iy, pz = *iz;
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {continue;}
    const double bx = tf_R00_*px + tf_R01_*py + tf_R02_*pz + tf_tx_;
    const double by = tf_R10_*px + tf_R11_*py + tf_R12_*pz + tf_ty_;
    const double bz = tf_R20_*px + tf_R21_*py + tf_R22_*pz + tf_tz_;
    if (bz < z_min || bz > z_max) {continue;}
    pts.push_back({bx, by});
  }

  esdf_grid_->update(pts);
  return true;
}

geometry_msgs::msg::PoseStamped ConstrainedController::getMotionTarget(
  double motion_target_dist,
  const nav_msgs::msg::Path & local_plan,
  int * sel_idx_out) const
{
  int idx = 0;
  for (size_t i = 0; i < local_plan.poses.size(); ++i) {
    const auto & p = local_plan.poses[i].pose.position;
    if (std::hypot(p.x, p.y) >= motion_target_dist) {
      idx = static_cast<int>(i);
      if (sel_idx_out) {*sel_idx_out = idx;}
      return local_plan.poses[i];
    }
  }
  idx = static_cast<int>(local_plan.poses.size()) - 1;
  if (sel_idx_out) {*sel_idx_out = idx;}
  return local_plan.poses.back();
}

geometry_msgs::msg::TwistStamped ConstrainedController::zeroTwist(
  const std::string & frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TwistStamped t;
  t.header.frame_id = frame;
  t.header.stamp    = stamp;
  return t;
}

geometry_msgs::msg::TwistStamped
ConstrainedController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;
  std::lock_guard<std::mutex> param_lock(param_handler_->getMutex());
  const auto * params    = param_handler_->getParams();
  const std::string base_frame = costmap_ros_->getBaseFrameID();
  const rclcpp::Time stamp     = pose.header.stamp;
  const uint64_t tick          = log_->newTick(stamp.seconds());

  // 1. Local plan slice in base_link (AMCL-free).
  nav_msgs::msg::Path local_plan;
  try {
    local_plan = path_handler_->getLocalPlan(pose, params->max_robot_pose_search_dist);
  } catch (const std::exception & e) {
    log_->event(std::string("getLocalPlan failed: ") + e.what());
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
      "computeVelocityCommands: %s", e.what());
    return zeroTwist(base_frame, stamp);
  }

  if (transformed_plan_pub_ && transformed_plan_pub_->is_activated()) {
    transformed_plan_pub_->publish(local_plan);
  }

  // 2. Lookahead point.
  int sel_idx = 0;
  const auto target_ps = getMotionTarget(params->motion_target_dist, local_plan, &sel_idx);

  if (motion_target_pub_ && motion_target_pub_->is_activated()) {
    geometry_msgs::msg::PointStamped pt;
    pt.header = target_ps.header;
    pt.point  = target_ps.pose.position;
    motion_target_pub_->publish(pt);
  }

  // Compute dist_to_goal using the same TF the controller server uses in
  // isGoalReached(): re-transform the goal from its original map frame to
  // odom with the CURRENT AMCL snapshot.  Using the plan-time snapshot
  // accumulates AMCL drift, causing the controller to declare "goal reached"
  // while the server's goal checker (which is always live) still sees the
  // robot as outside XY tolerance.
  const auto & odom_path = path_handler_->getPathInOdom();
  double dist_to_goal = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::PoseStamped robot_in_odom;
  if (!odom_path.poses.empty()) {
    try {
      const std::string odom_frame = path_handler_->getOdomFrame();

      // Robot → odom (EKF, same source as costmap::getRobotPose).
      geometry_msgs::msg::TransformStamped tf_base_to_odom;
      tf_base_to_odom = tf_buffer_->lookupTransform(
        odom_frame, pose.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(pose, robot_in_odom, tf_base_to_odom);

      // Goal → odom using live AMCL (same source as server's transformPose).
      const auto & goal_map = path_handler_->getGoalInMap();
      geometry_msgs::msg::PoseStamped goal_in_odom;
      geometry_msgs::msg::TransformStamped tf_map_to_odom;
      tf_map_to_odom = tf_buffer_->lookupTransform(
        odom_frame, goal_map.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(goal_map, goal_in_odom, tf_map_to_odom);

      dist_to_goal = std::hypot(
        robot_in_odom.pose.position.x - goal_in_odom.pose.position.x,
        robot_in_odom.pose.position.y - goal_in_odom.pose.position.y);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "dist_to_goal TF failed: %s", e.what());
    }
  }

  // Near-goal phase: within position tolerance, actively correct yaw to match
  // the planner's intended heading (= what the server's isGoalReached() checks).
  // Without this the robot stops with a heading mismatch and the goal checker
  // never fires, because validateOrientations only fixes intermediate poses —
  // the last pose keeps the planner yaw, which may differ from the arrival
  // travel direction.
  //
  // Threshold is 0.07 rad — just under yaw_goal_tolerance (0.10 rad) — so the
  // server fires immediately when we return zero.
  static constexpr double kNearGoalYawTol = 0.07;  // rad

  if (dist_to_goal < params->goal_dist_tolerance) {
    // Compute yaw error the same way isGoalReached() does:
    // robot_yaw_odom vs transform(goal_in_map, current_AMCL).yaw
    double yaw_err = 0.0;
    bool yaw_ok = false;
    if (!odom_path.poses.empty()) {
      try {
        const auto & goal_map = path_handler_->getGoalInMap();
        geometry_msgs::msg::PoseStamped goal_odom_for_yaw;
        geometry_msgs::msg::TransformStamped tf_m2o;
        tf_m2o = tf_buffer_->lookupTransform(
          path_handler_->getOdomFrame(), goal_map.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(goal_map, goal_odom_for_yaw, tf_m2o);
        const double goal_yaw  = tf2::getYaw(goal_odom_for_yaw.pose.orientation);
        const double robot_yaw = tf2::getYaw(robot_in_odom.pose.orientation);
        yaw_err = angles::shortest_angular_distance(robot_yaw, goal_yaw);
        yaw_ok  = std::abs(yaw_err) < kNearGoalYawTol;
      } catch (const tf2::TransformException &) {
        yaw_ok = true;  // TF failed: don't spin forever, just stop
      }
    } else {
      yaw_ok = true;
    }

    if (yaw_ok) {
      if (!goal_reached_) {log_->event("goal reached"); goal_reached_ = true;}
      auto stop = zeroTwist(base_frame, stamp);
      log_->logState(tick, stamp.seconds(), 0, 0, 0, stop.twist, stop.twist, false, dist_to_goal);
      return stop;
    }

    // Yaw not yet within tolerance: pure in-place rotation, no translation.
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
      "[CBF t=%lu] near-goal yaw correction: yaw_err=%.3f rad dist=%.3fm",
      tick, yaw_err, dist_to_goal);
    geometry_msgs::msg::TwistStamped yaw_cmd;
    yaw_cmd.header.frame_id = base_frame;
    yaw_cmd.header.stamp    = stamp;
    yaw_cmd.twist.angular.z =
      std::clamp(yaw_err * params->k_yaw, -params->v_angular_max, params->v_angular_max);
    return yaw_cmd;
  }

  // 3. Nominal P controller.
  NominalDebug nom_dbg;
  auto u_nom = nominal_->compute(target_ps.pose, &nom_dbg);

  // 4. Update ESDF from latest point cloud (horizontal beams only).
  {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud = last_cloud_;
    }
    if (cloud) {
      if (!updateEsdf(*cloud)) {
        log_->event("updateEsdf failed (TF lookup)");
      }
    } else {
      log_->event("no point cloud yet — CBF pass-through this tick");
    }
  }

  // Publish ESDF grid for RViz at ~2 Hz (every 5 ticks).
  if (esdf_pub_ && esdf_pub_->is_activated() && esdf_grid_->hasData()) {
    if (++esdf_pub_throttle_ % 5 == 0) {
      const int n       = esdf_grid_->gridSize();
      const double res  = esdf_grid_->resolution();
      const double half = esdf_grid_->halfSize();

      nav_msgs::msg::OccupancyGrid msg;
      msg.header.frame_id            = base_frame;
      msg.header.stamp               = stamp;
      msg.info.resolution            = static_cast<float>(res);
      msg.info.width                 = static_cast<uint32_t>(n);
      msg.info.height                = static_cast<uint32_t>(n);
      msg.info.origin.position.x     = -half;
      msg.info.origin.position.y     = -half;
      msg.info.origin.orientation.w  = 1.0;

      // Map distance → occupancy: 0 = free (far), 100 = obstacle (near).
      // Clamp display at 2 m so the gradient is visible in RViz.
      constexpr double kMaxDist = 2.0;
      const auto & dist = esdf_grid_->distGrid();
      msg.data.resize(static_cast<size_t>(n * n));
      for (int i = 0; i < n * n; ++i) {
        const double d = static_cast<double>(dist[i]);
        msg.data[i] = static_cast<int8_t>(
          std::clamp(static_cast<int>((1.0 - d / kMaxDist) * 100.0), 0, 100));
      }
      esdf_pub_->publish(msg);
    }
  }

  // 5. CBF safety filter.
  //
  // wz safety in narrow passages:
  // The full CBF lever arm (gx·py − gy·px)·wz is kinematically correct
  // but causes contradictory constraints when both walls are close and wz
  // is large — the QP becomes infeasible and outputs u*=0. The root cause
  // is that the robot shouldn't be rotating while inside a doorway; it
  // should align heading BEFORE entry, then translate through.
  //
  // Fix: when the previous tick's min_h was below a threshold (narrow
  // passage detected), suppress wz in u_nom before passing to the CBF.
  // The lever arm in the CBF is KEPT intact so the filter is still
  // kinematically correct for the wz that does reach it.
  bool wz_clamped = false;
  auto u_cbf_in = u_nom;
  constexpr double kNarrowHThresh = 0.15;  // metres
  if (last_min_h_ < kNarrowHThresh) {
    u_cbf_in.angular.z = 0.0;
    wz_clamped = true;
  }

  CbfFilterResult cbf_res;
  cbf_res.u = u_nom;  // pass-through when no ESDF data

  if (esdf_grid_->hasData()) {
    cbf_res = cbf_filter_->filter(*esdf_grid_, u_cbf_in);

    // Restore wz from u_nom — CBF only constrains translation while
    // inside narrow passage; wz passes through unconstrained.
    if (wz_clamped) {
      cbf_res.u.angular.z = std::clamp(
        u_nom.angular.z, -params->v_angular_max, params->v_angular_max);
    }

    const double u_star_norm = std::hypot(
      cbf_res.u.linear.x, cbf_res.u.linear.y, cbf_res.u.angular.z);
    const double u_nom_norm = std::hypot(
      u_nom.linear.x, u_nom.linear.y, u_nom.angular.z);

    bool fallback_fired = false;
    if (!cbf_res.qp.ok) {
      log_->event("QP infeasible — falling back to clamped u_nom");
      fallback_fired = true;
    } else if (u_star_norm < 0.05 && u_nom_norm > 0.05) {
      log_->event("CBF near-zero output — falling back to clamped u_nom");
      fallback_fired = true;
    }
    if (fallback_fired) {
      cbf_res.u.linear.x  = std::clamp(u_nom.linear.x,  -params->v_linear_max,  params->v_linear_max);
      cbf_res.u.linear.y  = std::clamp(u_nom.linear.y,  -params->v_lateral_max, params->v_lateral_max);
      cbf_res.u.angular.z = std::clamp(u_nom.angular.z, -params->v_angular_max, params->v_angular_max);
    }

    last_min_h_ = cbf_res.min_h;
  }

  // 6. Assemble output and log.
  geometry_msgs::msg::TwistStamped out;
  out.header.frame_id = base_frame;
  out.header.stamp    = stamp;
  out.twist           = cbf_res.u;

  log_->logState(
    tick, stamp.seconds(), 0, 0, 0,
    u_nom, out.twist, u_nom.linear.x < 0.0, dist_to_goal);

  log_->logPath(
    tick, stamp.seconds(),
    target_ps.pose.position.x, target_ps.pose.position.y,
    tf2::getYaw(target_ps.pose.orientation),
    nom_dbg.r, nom_dbg.s, nom_dbg.ramp, nom_dbg.yaw_err,
    sel_idx, static_cast<int>(local_plan.poses.size()));

  if (!cbf_res.constraints.empty()) {
    const Eigen::Vector3d u_nom_e(u_nom.linear.x, u_nom.linear.y, u_nom.angular.z);
    const Eigen::Vector3d u_fin_e(out.twist.linear.x, out.twist.linear.y, out.twist.angular.z);
    log_->logCbfConstraints(tick, stamp.seconds(), cbf_res.constraints, u_nom_e, u_fin_e);
    log_->logQp(
      tick, stamp.seconds(),
      cbf_res.qp.ok,
      static_cast<int>(cbf_res.constraints.size()),
      cbf_res.qp.n_active,
      cbf_res.solve_time_us,
      cbf_res.qp.deviation,
      cbf_res.qp.iterations);
  }

  // ---- Debug: console + RViz + rqt_plot ----

  // Collect per-corner h values for markers and text.
  std::array<double, 4> h_vals = {1.0, 1.0, 1.0, 1.0};
  for (const auto & c : cbf_res.constraints) {
    if (c.corner_id >= 1 && c.corner_id <= 4) {
      h_vals[c.corner_id - 1] = std::min(h_vals[c.corner_id - 1], c.h);
    }
  }

  // 1. Throttled one-line console summary — the fastest way to see state.
  //    Format: [CBF] MODE | u_nom=(vx,vy,wz) u*=(vx,vy,wz) | h_min=X cst=N dev=X | dist=Xm
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
    "[CBF t=%lu] %s | "
    "u_nom=(%.2f,%.2f,%.2f) u*=(%.2f,%.2f,%.2f) | "
    "h_min=%.3f cst=%zu dev=%.3f | dist=%.2fm",
    tick,
    wz_clamped ? "NARROW(wz=0)" : "NORMAL      ",
    u_nom.linear.x, u_nom.linear.y, u_nom.angular.z,
    out.twist.linear.x, out.twist.linear.y, out.twist.angular.z,
    cbf_res.min_h,
    cbf_res.constraints.size(),
    cbf_res.qp.deviation,
    dist_to_goal);

  // 2. Warn immediately on abnormal conditions — always visible.
  const double out_norm = std::hypot(
    out.twist.linear.x, out.twist.linear.y, out.twist.angular.z);
  const double nom_norm = std::hypot(
    u_nom.linear.x, u_nom.linear.y, u_nom.angular.z);

  if (out_norm < 0.02 && nom_norm > 0.05) {
    RCLCPP_WARN(logger_,
      "[CBF t=%lu] *** ZERO OUTPUT with nonzero u_nom=%.3f *** "
      "h_min=%.3f qp_ok=%d wz_clamped=%d n_cst=%zu",
      tick, nom_norm, cbf_res.min_h,
      cbf_res.qp.ok ? 1 : 0,
      wz_clamped ? 1 : 0,
      cbf_res.constraints.size());
  }

  if (!cbf_res.qp.ok) {
    RCLCPP_WARN(logger_,
      "[CBF t=%lu] QP INFEASIBLE — h_min=%.3f n_cst=%zu wz_clamp=%d",
      tick, cbf_res.min_h,
      cbf_res.constraints.size(),
      wz_clamped ? 1 : 0);
  }

  // 3. Body corners in RViz — sphere colour shows safety margin.
  //    Green h>0.10m  Orange 0.02-0.10m  Red <0.02m (approaching limit)
  //    Text label shows h in metres so you can read it in RViz.
  if (corners_pub_ && corners_pub_->is_activated()) {
    visualization_msgs::msg::MarkerArray ma;
    const auto & corners = cbf_res.corners;
    static const char * const kCornerNames[4] = {
      "FR", "BR", "BL", "FL"};

    for (int i = 0; i < 4; ++i) {
      // Sphere
      visualization_msgs::msg::Marker sph;
      sph.header.frame_id = base_frame;
      sph.header.stamp    = stamp;
      sph.ns   = "cbf_corners";
      sph.id   = i;
      sph.type = visualization_msgs::msg::Marker::SPHERE;
      sph.action = visualization_msgs::msg::Marker::ADD;
      sph.pose.position.x = corners[i].x;
      sph.pose.position.y = corners[i].y;
      sph.pose.position.z = 0.05;
      sph.pose.orientation.w = 1.0;
      sph.scale.x = sph.scale.y = sph.scale.z = 0.10;
      const double h = h_vals[i];
      if (h > 0.10) {
        sph.color.r = 0.0f; sph.color.g = 1.0f; sph.color.b = 0.0f; sph.color.a = 1.0f;
      } else if (h > 0.02) {
        sph.color.r = 1.0f; sph.color.g = 0.55f; sph.color.b = 0.0f; sph.color.a = 1.0f;
      } else {
        sph.color.r = 1.0f; sph.color.g = 0.0f; sph.color.b = 0.0f; sph.color.a = 1.0f;
      }
      sph.lifetime = rclcpp::Duration::from_seconds(0.3);
      ma.markers.push_back(sph);

      // Text label: "FR\nh=0.034"
      visualization_msgs::msg::Marker txt;
      txt.header  = sph.header;
      txt.ns      = "cbf_corner_labels";
      txt.id      = i;
      txt.type    = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt.action  = visualization_msgs::msg::Marker::ADD;
      txt.pose.position.x = corners[i].x;
      txt.pose.position.y = corners[i].y;
      txt.pose.position.z = 0.25;
      txt.pose.orientation.w = 1.0;
      txt.scale.z = 0.08;
      txt.color.r = 1.0f; txt.color.g = 1.0f; txt.color.b = 1.0f; txt.color.a = 1.0f;
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%s\nh=%.3f", kCornerNames[i], h);
      txt.text    = buf;
      txt.lifetime = rclcpp::Duration::from_seconds(0.3);
      ma.markers.push_back(txt);
    }
    corners_pub_->publish(ma);
  }

  // 4. Float32MultiArray for rqt_plot.
  //    Add to rqt_plot: /constrained_controller/debug/data[0..10]
  //    [0]=h_min  [1]=qp_dev  [2]=n_cst
  //    [3]=vx_nom [4]=vy_nom  [5]=wz_nom
  //    [6]=vx*    [7]=vy*     [8]=wz*
  //    [9]=qp_infeasible(0/1) [10]=wz_clamped(0/1)
  if (debug_pub_ && debug_pub_->is_activated()) {
    std_msgs::msg::Float32MultiArray dbg;
    dbg.data = {
      static_cast<float>(cbf_res.min_h),
      static_cast<float>(cbf_res.qp.deviation),
      static_cast<float>(cbf_res.constraints.size()),
      static_cast<float>(u_nom.linear.x),
      static_cast<float>(u_nom.linear.y),
      static_cast<float>(u_nom.angular.z),
      static_cast<float>(out.twist.linear.x),
      static_cast<float>(out.twist.linear.y),
      static_cast<float>(out.twist.angular.z),
      static_cast<float>(cbf_res.qp.ok ? 0.0f : 1.0f),
      static_cast<float>(wz_clamped ? 1.0f : 0.0f)
    };
    debug_pub_->publish(dbg);
  }

  // ---- Stuck detection ----
  // Compare what we commanded vs what the robot actually did (from odometry).
  // If we commanded significant motion but the robot barely moved for
  // kStuckTicks consecutive ticks, it has hit an obstacle the ESDF cannot see.
  // Stop commanding and warn — the recovery is handled by Nav2's progress checker.
  constexpr double kCommandedThresh = 0.05;   // m/s — "significant command"
  constexpr double kActualThresh    = 0.02;   // m/s — "essentially not moving"
  constexpr int    kStuckTicks      = 20;     // 2 seconds at 10 Hz

  const double commanded_norm = std::hypot(
    out.twist.linear.x, out.twist.linear.y, out.twist.angular.z);
  const double actual_norm = std::hypot(
    velocity.linear.x, velocity.linear.y, velocity.angular.z);

  if (commanded_norm > kCommandedThresh && actual_norm < kActualThresh) {
    ++stuck_ticks_;
  } else {
    stuck_ticks_ = 0;
    stuck_       = false;
  }

  if (stuck_ticks_ >= kStuckTicks && !stuck_) {
    stuck_ = true;
    RCLCPP_WARN(logger_,
      "[CBF t=%lu] *** STUCK *** commanded=%.3f m/s actual=%.3f m/s for %d ticks. "
      "Obstacle not visible in ESDF (h_min=%.3f). Stopping — Nav2 recovery will take over.",
      tick, commanded_norm, actual_norm, stuck_ticks_, cbf_res.min_h);
    log_->event("STUCK — undetected obstacle, stopping for Nav2 recovery");
  }

  if (stuck_) {
    return zeroTwist(base_frame, stamp);
  }

  return out;
}

}  // namespace nav2_constrained_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_constrained_controller::ConstrainedController,
  nav2_core::Controller)
