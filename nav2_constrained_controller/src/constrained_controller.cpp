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

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void ConstrainedController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_      = parent;
  plugin_name_ = name;
  tf_buffer_   = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent.lock();
  if (!node) {throw std::runtime_error("ConstrainedController: invalid parent node");}
  logger_ = node->get_logger();
  clock_  = node->get_clock();

  odom_frame_ = costmap_ros_->getGlobalFrameID();

  param_handler_ = std::make_unique<ParameterHandler>(parent, plugin_name_, logger_);
  auto * params  = param_handler_->getParams();

  const tf2::Duration transform_tol = tf2::durationFromSec(0.1);
  path_handler_ = std::make_unique<PathHandler>(transform_tol, tf_buffer_, costmap_ros_);
  esdf_grid_    = std::make_unique<EsdfGrid>(params);
  cbf_filter_   = std::make_unique<CBFSafetyFilter>(params);
  log_          = std::make_unique<Logger>();

  esdf_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "constrained_controller/esdf_grid", rclcpp::SystemDefaultsQoS());
  corners_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "constrained_controller/body_corners", rclcpp::SystemDefaultsQoS());
  velocity_arrow_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "constrained_controller/velocity_arrows", rclcpp::SystemDefaultsQoS());
  debug_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>(
    "constrained_controller/debug", rclcpp::SystemDefaultsQoS());
  cbf_constraints_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>(
    "constrained_controller/cbf_constraints", rclcpp::SystemDefaultsQoS());
  cbf_gradients_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "constrained_controller/cbf_gradients", rclcpp::SystemDefaultsQoS());
  tracking_target_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "constrained_controller/tracking_target", 1);

  RCLCPP_INFO(logger_,
    "ConstrainedController '%s' configured (PD lattice tracker + CBF). cloud=%s log=%s "
    "lookahead=%.2fm kp_pos=%.2f kp_yaw=%.2f",
    plugin_name_.c_str(),
    params->pointcloud_topic.c_str(),
    params->log_dir.c_str(),
    params->lookahead_dist,
    params->kp_pos,
    params->kp_yaw);
}

void ConstrainedController::activate()
{
  auto node = parent_.lock();
  if (!node) {return;}
  auto * params = param_handler_->getParams();

  log_->open(params->log_dir, params->log_enabled);
  log_->event("activate");

  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    params->pointcloud_topic, rclcpp::SensorDataQoS(),
    std::bind(&ConstrainedController::pointcloudCallback, this, std::placeholders::_1));

  esdf_pub_->on_activate();
  corners_pub_->on_activate();
  velocity_arrow_pub_->on_activate();
  debug_pub_->on_activate();
  cbf_constraints_pub_->on_activate();
  cbf_gradients_pub_->on_activate();
  tracking_target_pub_->on_activate();

  goal_reached_          = false;
  esdf_pub_throttle_     = 0;
  has_lidar_tf_          = false;
  last_min_h_            = 1.0;
  stuck_                 = false;
  stuck_start_x_         = 0.0;
  stuck_start_y_         = 0.0;
  stuck_start_yaw_       = 0.0;
  stuck_start_time_      = rclcpp::Time(0, 0, RCL_ROS_TIME);
  has_plan_              = false;
  near_goal_yaw_active_  = false;
  near_goal_yaw_start_   = rclcpp::Time(0, 0, RCL_ROS_TIME);
  resetTrackerState();
}

void ConstrainedController::deactivate()
{
  if (log_) {log_->event("deactivate"); log_->close();}
  cloud_sub_.reset();
  if (esdf_pub_)            {esdf_pub_->on_deactivate();}
  if (corners_pub_)         {corners_pub_->on_deactivate();}
  if (velocity_arrow_pub_)  {velocity_arrow_pub_->on_deactivate();}
  if (debug_pub_)           {debug_pub_->on_deactivate();}
  if (cbf_constraints_pub_) {cbf_constraints_pub_->on_deactivate();}
  if (cbf_gradients_pub_)   {cbf_gradients_pub_->on_deactivate();}
  if (tracking_target_pub_) {tracking_target_pub_->on_deactivate();}
}

void ConstrainedController::cleanup()
{
  param_handler_.reset();
  path_handler_.reset();
  esdf_grid_.reset();
  cbf_filter_.reset();
  log_.reset();
  esdf_pub_.reset();
  corners_pub_.reset();
  velocity_arrow_pub_.reset();
  debug_pub_.reset();
  cbf_constraints_pub_.reset();
  cbf_gradients_pub_.reset();
  tracking_target_pub_.reset();
}

void ConstrainedController::setPlan(const nav_msgs::msg::Path & path)
{
  const bool ok = path_handler_->setPlan(path);
  goal_reached_         = false;
  stuck_                = false;
  stuck_start_x_        = 0.0;
  stuck_start_y_        = 0.0;
  stuck_start_yaw_      = 0.0;
  stuck_start_time_     = rclcpp::Time(0, 0, RCL_ROS_TIME);
  has_plan_             = false;
  near_goal_yaw_active_ = false;
  esdf_grid_->reset();
  // Fresh plan → fresh PD memory (the target may jump; a stale D would spike).
  resetTrackerState();

  if (!path.poses.empty()) {
    goal_in_map_ = path.poses.back();
    goal_in_map_.header.frame_id = path.header.frame_id;
    has_plan_ = true;
  }

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
    p->v_linear_max  = p->v_linear_max_initial;
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

// ---------------------------------------------------------------------------
// LiDAR → ESDF
// ---------------------------------------------------------------------------

void ConstrainedController::pointcloudCallback(
  sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  last_cloud_ = msg;
}

bool ConstrainedController::updateEsdf(const sensor_msgs::msg::PointCloud2 & cloud)
{
  const std::string base_frame = costmap_ros_->getBaseFrameID();
  if (!has_lidar_tf_) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        base_frame, cloud.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
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

  // ── Near-body 3D probe (collision diagnostic) ─────────────────────────────
  // Examine RAW cloud points (all heights) inside the body footprint (+margin)
  // in XY. The ESDF z-slice below drops everything outside [z_min, z_max]; this
  // probe records what the slice throws away near the body so a collision with
  // an out-of-slice obstacle is conclusively attributable.
  const double Lext = 0.5 * params->footprint_length + params->footprint_dl;
  const double db   = params->footprint_db;
  constexpr double kProbeMargin = 0.15;   // m of XY slack around the body
  NearBodyProbe probe{};   // fresh each call

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

    // Probe: is this point near the body in XY (any height)?
    if (std::abs(bx) <= Lext + kProbeMargin && std::abs(by) <= db + kProbeMargin) {
      if (probe.n_near == 0) {probe.min_z = bz; probe.max_z = bz;}
      else {probe.min_z = std::min(probe.min_z, bz); probe.max_z = std::max(probe.max_z, bz);}
      ++probe.n_near;
      const bool below = bz < z_min;
      const bool above = bz > z_max;
      if (below) {++probe.n_below;}
      else if (above) {++probe.n_above;}
      else {++probe.n_in;}
      if (below || above) {   // out-of-slice → invisible to the CBF/ESDF
        const double d_edge = std::hypot(
          std::max(0.0, std::abs(bx) - Lext), std::max(0.0, std::abs(by) - db));
        if (d_edge < probe.nearest_oob_dist) {
          probe.nearest_oob_dist = d_edge;
          probe.nearest_oob_x = bx; probe.nearest_oob_y = by; probe.nearest_oob_z = bz;
        }
      }
    }

    if (bz < z_min || bz > z_max) {continue;}
    pts.push_back({bx, by});
  }
  probe.valid = true;
  near_body_probe_ = probe;
  esdf_grid_->update(pts);
  return true;
}

geometry_msgs::msg::TwistStamped ConstrainedController::zeroTwist(
  const std::string & frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TwistStamped t;
  t.header.frame_id = frame;
  t.header.stamp    = stamp;
  return t;
}

// ---------------------------------------------------------------------------
// PD path tracker
// ---------------------------------------------------------------------------

void ConstrainedController::resetTrackerState()
{
  prev_err_ox_       = 0.0;
  prev_err_oy_       = 0.0;
  have_prev_err_pos_ = false;
  prev_pos_stamp_    = rclcpp::Time(0, 0, RCL_ROS_TIME);
  prev_yaw_err_      = 0.0;
  have_prev_yaw_err_ = false;
  prev_yaw_stamp_    = rclcpp::Time(0, 0, RCL_ROS_TIME);
  prev_track_mode_   = PathHandler::TRACK_NORMAL;
}

bool ConstrainedController::trackPath(
  const geometry_msgs::msg::PoseStamped & robot_in_odom,
  const rclcpp::Time & stamp,
  geometry_msgs::msg::Twist & u_nom,
  PathHandler::TrackingTarget & target_out,
  double & yaw_err_out,
  double & v_scale_out)
{
  const auto * params = param_handler_->getParams();

  if (!path_handler_->getTrackingTarget(
      robot_in_odom, params->lookahead_dist,
      params->max_robot_pose_search_dist, target_out))
  {
    return false;
  }

  const double rx   = robot_in_odom.pose.position.x;
  const double ry   = robot_in_odom.pose.position.y;
  const double ryaw = tf2::getYaw(robot_in_odom.pose.orientation);

  // ── Position PD (odom frame) ──────────────────────────────────────────────
  const double ex_o = target_out.pose.pose.position.x - rx;
  const double ey_o = target_out.pose.pose.position.y - ry;

  double dex_o = 0.0, dey_o = 0.0;
  if (have_prev_err_pos_) {
    const double dt = (stamp - prev_pos_stamp_).seconds();
    if (dt > 1e-3) {
      dex_o = (ex_o - prev_err_ox_) / dt;
      dey_o = (ey_o - prev_err_oy_) / dt;
    }
  }
  prev_err_ox_       = ex_o;
  prev_err_oy_       = ey_o;
  prev_pos_stamp_    = stamp;
  have_prev_err_pos_ = true;

  const double ux_o = params->kp_pos * ex_o + params->kd_pos * dex_o;
  const double uy_o = params->kp_pos * ey_o + params->kd_pos * dey_o;

  // Rotate odom-frame command into base_link and clamp per axis.
  const double c = std::cos(-ryaw), s = std::sin(-ryaw);
  u_nom.linear.x = std::clamp(
    c * ux_o - s * uy_o, -params->v_linear_max, params->v_linear_max);
  u_nom.linear.y = std::clamp(
    s * ux_o + c * uy_o, -params->v_lateral_max, params->v_lateral_max);

  // ── Yaw PD — track the LATTICE's planned yaw at the target pose ──────────
  // The OMNI lattice plans yaw as a search dimension; the path orientations
  // are footprint-validated intent (alley-axis hold, pre-rotations at doors).
  const double yaw_target = tf2::getYaw(target_out.pose.pose.orientation);
  const double yaw_err    = angles::shortest_angular_distance(ryaw, yaw_target);

  double dyaw_err = 0.0;
  if (have_prev_yaw_err_) {
    const double dt = (stamp - prev_yaw_stamp_).seconds();
    if (dt > 1e-3) {
      dyaw_err = angles::shortest_angular_distance(prev_yaw_err_, yaw_err) / dt;
    }
  }
  prev_yaw_err_      = yaw_err;
  prev_yaw_stamp_    = stamp;
  have_prev_yaw_err_ = true;

  u_nom.angular.z = std::clamp(
    params->kp_yaw * yaw_err + params->kd_yaw * dyaw_err,
    -params->v_angular_max, params->v_angular_max);

  // ── Yaw–translation coupling (cos taper) ──────────────────────────────────
  // Scale the WHOLE translation vector (both axes — direction preserved) by
  // how well the body is aligned: full speed at yaw_err = 0, standstill at
  // ≥ 90°. wz keeps full authority — it is what shrinks the error that is
  // causing the slowdown, so the taper self-releases. During APPROACH_ROTATION
  // the clamped target carries the pre-rotation yaw, so yaw_err ≈ 0 and the
  // taper stays inactive until the robot is standing on the validated spot.
  const double v_scale = std::max(0.0, std::cos(yaw_err));
  u_nom.linear.x *= v_scale;
  u_nom.linear.y *= v_scale;

  yaw_err_out = yaw_err;
  v_scale_out = v_scale;
  return true;
}

// ---------------------------------------------------------------------------
// Main control loop
// ---------------------------------------------------------------------------

geometry_msgs::msg::TwistStamped
ConstrainedController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;
  std::lock_guard<std::mutex> param_lock(param_handler_->getMutex());
  const auto * params     = param_handler_->getParams();
  const std::string base_frame = costmap_ros_->getBaseFrameID();
  const rclcpp::Time stamp     = pose.header.stamp;
  const uint64_t tick          = log_->newTick(stamp.seconds());

  // ── 1. ESDF update (tracker viz + CBF need fresh data) ──────────────────
  {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    {std::lock_guard<std::mutex> lock(cloud_mutex_); cloud = last_cloud_;}
    if (cloud) {
      if (!updateEsdf(*cloud)) {log_->event("updateEsdf failed (TF)");}
    } else {
      log_->event("no point cloud yet — pass-through");
    }
  }

  // Near-body 3D profile (collision diagnostic). Logged every tick a cloud was
  // processed, before any control branch returns, so the wedge/stuck ticks are
  // captured. nearest_oob_dist = -1 when no out-of-slice point is near the body.
  if (near_body_probe_.valid) {
    const auto & pr = near_body_probe_;
    log_->logEsdf3d(tick, stamp.seconds(),
      pr.n_near, pr.n_below, pr.n_in, pr.n_above,
      pr.n_near > 0 ? pr.min_z : 0.0, pr.n_near > 0 ? pr.max_z : 0.0,
      pr.nearest_oob_dist > 1e8 ? -1.0 : pr.nearest_oob_dist,
      pr.nearest_oob_x, pr.nearest_oob_y, pr.nearest_oob_z);
  }

  // ── 2. dist_to_goal via live AMCL TF ─────────────────────────────────────
  double dist_to_goal = std::numeric_limits<double>::infinity();
  // T_odom_from_map per-tick snapshot for drift analysis. Default to NaN —
  // gets filled in below if the TF lookup succeeds.
  constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
  double tx_o_m = NaN, ty_o_m = NaN, ryaw_o_m = NaN;
  geometry_msgs::msg::PoseStamped robot_in_odom;
  const auto & odom_path = path_handler_->getPathInOdom();
  if (!odom_path.poses.empty()) {
    try {
      const std::string odom_frame = path_handler_->getOdomFrame();
      auto tf_b2o = tf_buffer_->lookupTransform(
        odom_frame, pose.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(pose, robot_in_odom, tf_b2o);

      geometry_msgs::msg::PoseStamped goal_in_odom;
      const auto & goal_map = path_handler_->getGoalInMap();
      auto tf_m2o = tf_buffer_->lookupTransform(
        odom_frame, goal_map.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(goal_map, goal_in_odom, tf_m2o);

      // Snapshot T_odom_from_map at this tick. Variation over time of
      // (tx_o_m, ty_o_m, ryaw_o_m) reveals AMCL drift / relocalize events.
      // Difference vs setplan-time T_odom_from_map = how far the path-in-odom
      // is geometrically off relative to the real-world walls.
      tx_o_m   = tf_m2o.transform.translation.x;
      ty_o_m   = tf_m2o.transform.translation.y;
      ryaw_o_m = tf2::getYaw(tf_m2o.transform.rotation);

      dist_to_goal = std::hypot(
        robot_in_odom.pose.position.x - goal_in_odom.pose.position.x,
        robot_in_odom.pose.position.y - goal_in_odom.pose.position.y);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "dist_to_goal TF failed: %s", e.what());
    }
  }

  // ── 3. Near-goal: yaw alignment then stop ────────────────────────────────
  static constexpr double kNearGoalYawTol = 0.07;
  static constexpr double kYawTimeoutS    = 5.0;
  if (dist_to_goal < params->goal_dist_tolerance) {
    // Once goal_reached_ is set (yaw ok OR timeout), stay stopped permanently.
    // Without this, the yaw correction restarts every tick causing an infinite loop
    // when CBF blocks rotation near walls (lever × wz > γ × h_min).
    if (goal_reached_) {
      return zeroTwist(base_frame, stamp);
    }
    double yaw_err = 0.0;
    bool yaw_ok = false;
    if (!odom_path.poses.empty()) {
      try {
        const auto & goal_map = path_handler_->getGoalInMap();
        geometry_msgs::msg::PoseStamped goal_odom;
        auto tf_m2o = tf_buffer_->lookupTransform(
          path_handler_->getOdomFrame(), goal_map.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(goal_map, goal_odom, tf_m2o);
        yaw_err = angles::shortest_angular_distance(
          tf2::getYaw(robot_in_odom.pose.orientation),
          tf2::getYaw(goal_odom.pose.orientation));
        yaw_ok = std::abs(yaw_err) < kNearGoalYawTol;
      } catch (const tf2::TransformException &) { yaw_ok = true; }
    } else { yaw_ok = true; }

    if (yaw_ok || !near_goal_yaw_active_) {
      near_goal_yaw_start_  = stamp;
      near_goal_yaw_active_ = !yaw_ok;
    }
    if ((stamp - near_goal_yaw_start_).seconds() > kYawTimeoutS && !yaw_ok) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
        "[CBF t=%lu] near-goal yaw timeout (err=%.3f rad) — stopping", tick, yaw_err);
      yaw_ok = true;
    }

    if (yaw_ok) {
      if (!goal_reached_) {log_->event("goal reached"); goal_reached_ = true;}
      auto stop = zeroTwist(base_frame, stamp);
      log_->logState(tick, stamp.seconds(), 0, 0, 0, stop.twist, stop.twist, false,
                     dist_to_goal, tx_o_m, ty_o_m, ryaw_o_m);
      return stop;
    }

    RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
      "[CBF t=%lu] near-goal yaw: err=%.3f rad dist=%.3fm", tick, yaw_err, dist_to_goal);
    geometry_msgs::msg::Twist yaw_twist;
    yaw_twist.angular.z =
      std::clamp(yaw_err * params->k_yaw, -params->v_angular_max, params->v_angular_max);
    if (esdf_grid_->hasData()) {
      auto cbf_yaw = cbf_filter_->filter(*esdf_grid_, yaw_twist);
      if (cbf_yaw.qp.ok) {
        last_min_h_ = cbf_yaw.min_h;
        geometry_msgs::msg::TwistStamped yaw_cmd;
        yaw_cmd.header.frame_id = base_frame;
        yaw_cmd.header.stamp    = stamp;
        yaw_cmd.twist           = cbf_yaw.u;
        return yaw_cmd;
      }
    }
    geometry_msgs::msg::TwistStamped yaw_cmd;
    yaw_cmd.header.frame_id = base_frame;
    yaw_cmd.header.stamp    = stamp;
    yaw_cmd.twist           = yaw_twist;
    return yaw_cmd;
  }

  // ── 4. PD tracker on the lattice path → u_nom ────────────────────────────
  geometry_msgs::msg::Twist u_nom;
  PathHandler::TrackingTarget target;
  double yaw_err = 0.0;
  double v_scale = 1.0;
  bool track_ok = false;
  if (!odom_path.poses.empty()) {
    track_ok = trackPath(robot_in_odom, stamp, u_nom, target, yaw_err, v_scale);
  }
  if (!track_ok) {
    // No plan or no target — refuse to move.
    log_->event("trackPath returned false — zero output");
    auto stop = zeroTwist(base_frame, stamp);
    log_->logState(tick, stamp.seconds(), 0, 0, 0, stop.twist, stop.twist, false,
                   dist_to_goal, tx_o_m, ty_o_m, ryaw_o_m);
    return stop;
  }

  // One event per tracker-mode transition so the rotation phases are
  // reconstructible from ctrl_events alone.
  if (target.mode != prev_track_mode_) {
    static const char * const kModeNames[3] =
    {"NORMAL", "APPROACH_ROTATION", "ROTATE_IN_PLACE"};
    char ev[160];
    std::snprintf(ev, sizeof(ev),
      "tracker mode %s -> %s | target=(%.2f,%.2f) target_yaw=%.2f "
      "robot_yaw=%.2f yaw_err=%.2f dist=%.3f",
      kModeNames[std::clamp(prev_track_mode_, 0, 2)],
      kModeNames[std::clamp(target.mode, 0, 2)],
      target.pose.pose.position.x, target.pose.pose.position.y,
      tf2::getYaw(target.pose.pose.orientation),
      tf2::getYaw(robot_in_odom.pose.orientation),
      yaw_err, target.dist_to_target);
    log_->event(ev);
    RCLCPP_INFO(logger_, "[PD t=%lu] %s", tick, ev);
    prev_track_mode_ = target.mode;
  }

  // Publish ESDF OccupancyGrid at ~2 Hz.
  if (esdf_pub_ && esdf_pub_->is_activated() && esdf_grid_->hasData()) {
    if (++esdf_pub_throttle_ % 5 == 0) {
      const int n = esdf_grid_->gridSize();
      const double res = esdf_grid_->resolution(), half = esdf_grid_->halfSize();
      nav_msgs::msg::OccupancyGrid msg;
      msg.header.frame_id = base_frame; msg.header.stamp = stamp;
      msg.info.resolution = static_cast<float>(res);
      msg.info.width = msg.info.height = static_cast<uint32_t>(n);
      msg.info.origin.position.x = -half; msg.info.origin.position.y = -half;
      msg.info.origin.orientation.w = 1.0;
      constexpr double kMaxDist = 2.0;
      const auto & dist = esdf_grid_->distGrid();
      msg.data.resize(static_cast<size_t>(n * n));
      for (int i = 0; i < n * n; ++i) {
        msg.data[i] = static_cast<int8_t>(
          std::clamp(static_cast<int>((1.0 - dist[i] / kMaxDist) * 100.0), 0, 100));
      }
      esdf_pub_->publish(msg);
    }
  }

  // ── 5. CBF safety filter ──────────────────────────────────────────────────
  CbfFilterResult cbf_res;
  cbf_res.u = u_nom;
  if (esdf_grid_->hasData()) {
    cbf_res = cbf_filter_->filter(*esdf_grid_, u_nom);
    if (!cbf_res.qp.ok) {
      log_->event("QP box-infeasible — zeroing for safety");
      cbf_res.u.linear.x = cbf_res.u.linear.y = cbf_res.u.angular.z = 0.0;
    }
    if (cbf_res.qp.max_slack > 0.01) {
      log_->event("CBF slack: " + std::to_string(cbf_res.qp.max_slack) + " m/s");
    }
    last_min_h_ = cbf_res.min_h;
  }

  // ── 6. Assemble output ────────────────────────────────────────────────────
  geometry_msgs::msg::TwistStamped out;
  out.header.frame_id = base_frame;
  out.header.stamp    = stamp;
  out.twist           = cbf_res.u;

  // Publish velocity vectors at base_link origin: u_nom (blue, before CBF) and
  // u* (green, after CBF).  Length scaled to 0.5 m / (m/s) for visibility;
  // angular component drawn as a small purple arc at robot centre.
  if (velocity_arrow_pub_ && velocity_arrow_pub_->is_activated()) {
    visualization_msgs::msg::MarkerArray ma;
    constexpr double kVelScale = 0.5;
    auto makeArrow = [&](const char * ns, int id,
                          double vx, double vy,
                          float r, float g, float b) {
      visualization_msgs::msg::Marker arr;
      arr.header.frame_id = base_frame; arr.header.stamp = stamp;
      arr.ns = ns; arr.id = id;
      arr.type = visualization_msgs::msg::Marker::ARROW;
      arr.action = visualization_msgs::msg::Marker::ADD;
      geometry_msgs::msg::Point ps, pe;
      ps.x = 0.0; ps.y = 0.0; ps.z = 0.10;
      pe.x = kVelScale * vx; pe.y = kVelScale * vy; pe.z = 0.10;
      arr.points = {ps, pe};
      arr.scale.x = 0.025; arr.scale.y = 0.05; arr.scale.z = 0.0;
      arr.color.r = r; arr.color.g = g; arr.color.b = b; arr.color.a = 0.9f;
      arr.lifetime = rclcpp::Duration::from_seconds(0.3);
      return arr;
    };
    ma.markers.push_back(makeArrow("u_nom", 0,
      u_nom.linear.x, u_nom.linear.y, 0.1f, 0.4f, 1.0f));   // blue
    ma.markers.push_back(makeArrow("u_star", 0,
      out.twist.linear.x, out.twist.linear.y, 0.1f, 1.0f, 0.4f));   // green
    velocity_arrow_pub_->publish(ma);
  }

  // ── 7. Tracking-target viz (odom → base_link) ────────────────────────────
  const double rx_o   = robot_in_odom.pose.position.x;
  const double ry_o   = robot_in_odom.pose.position.y;
  const double ryaw_o = tf2::getYaw(robot_in_odom.pose.orientation);
  if (tracking_target_pub_ && tracking_target_pub_->is_activated()) {
    const double cR = std::cos(-ryaw_o), sR = std::sin(-ryaw_o);
    const double dx = target.pose.pose.position.x - rx_o;
    const double dy = target.pose.pose.position.y - ry_o;
    geometry_msgs::msg::PointStamped pt;
    pt.header.frame_id = base_frame; pt.header.stamp = stamp;
    pt.point.x = cR * dx - sR * dy;
    pt.point.y = sR * dx + cR * dy;
    tracking_target_pub_->publish(pt);
  }

  // ── 8. Logging ────────────────────────────────────────────────────────────
  const double v_ref_mag = std::hypot(u_nom.linear.x, u_nom.linear.y);
  log_->logState(tick, stamp.seconds(), rx_o, ry_o, ryaw_o,
    u_nom, out.twist, false, dist_to_goal, tx_o_m, ty_o_m, ryaw_o_m);
  log_->logPath(tick, stamp.seconds(),
    target.pose.pose.position.x, target.pose.pose.position.y,
    tf2::getYaw(target.pose.pose.orientation),
    v_ref_mag, target.dist_to_target, target.cross_track,
    yaw_err, static_cast<int>(target.target_idx),
    static_cast<int>(target.n_remaining),
    target.mode, v_scale);
  if (!cbf_res.constraints.empty()) {
    const Eigen::Vector3d un(u_nom.linear.x, u_nom.linear.y, u_nom.angular.z);
    const Eigen::Vector3d uf(out.twist.linear.x, out.twist.linear.y, out.twist.angular.z);
    log_->logCbfConstraints(tick, stamp.seconds(), cbf_res.constraints, un, uf);
    log_->logQp(tick, stamp.seconds(), cbf_res.qp.ok,
      static_cast<int>(cbf_res.constraints.size()), cbf_res.qp.n_active,
      cbf_res.solve_time_us, cbf_res.qp.deviation, cbf_res.qp.iterations);
  }

  // ── 9. Debug console + RViz + rqt_plot ────────────────────────────────────
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
    "[PD t=%lu] m=%d tgt=(%.2f,%.2f) xtrack=%.3f yaw_err=%.2f vscale=%.2f "
    "v_ref=%.2f | u_nom=(%.2f,%.2f,%.2f) u*=(%.2f,%.2f,%.2f) | "
    "h_min=%.3f cst=%zu dev=%.3f | dist=%.2fm remain=%zu",
    tick, target.mode, target.pose.pose.position.x, target.pose.pose.position.y,
    target.cross_track, yaw_err, v_scale, v_ref_mag,
    u_nom.linear.x, u_nom.linear.y, u_nom.angular.z,
    out.twist.linear.x, out.twist.linear.y, out.twist.angular.z,
    cbf_res.min_h, cbf_res.constraints.size(),
    cbf_res.qp.deviation, dist_to_goal, target.n_remaining);

  const double out_norm = std::hypot(out.twist.linear.x, out.twist.linear.y, out.twist.angular.z);
  const double nom_norm = std::hypot(u_nom.linear.x, u_nom.linear.y, u_nom.angular.z);
  if (out_norm < 0.02 && nom_norm > 0.05) {
    RCLCPP_WARN(logger_,
      "[CBF t=%lu] *** ZERO OUTPUT *** h_min=%.3f qp_ok=%d n_cst=%zu slack=%.4f",
      tick, cbf_res.min_h, cbf_res.qp.ok ? 1 : 0,
      cbf_res.constraints.size(), cbf_res.qp.max_slack);
  }
  if (!cbf_res.qp.ok) {
    RCLCPP_WARN(logger_,
      "[CBF t=%lu] QP box-infeasible — h_min=%.3f n_cst=%zu",
      tick, cbf_res.min_h, cbf_res.constraints.size());
  }

  // Constraint debug: grouped summary.
  {
    constexpr int kMaxSteps = 3;
    struct GroupInfo {
      int count{0}; double worst_margin{1e9}; const CbfConstraint * worst{nullptr};
      int n_violated{0};
    };
    std::array<GroupInfo, kMaxSteps> groups;
    for (const auto & c : cbf_res.constraints) {
      int s = std::clamp(c.predict_step, 0, kMaxSteps - 1);
      groups[s].count++;
      if (c.margin < groups[s].worst_margin) {
        groups[s].worst_margin = c.margin; groups[s].worst = &c;
      }
      if (c.Au > c.rhs + 1e-3) {groups[s].n_violated++;}
    }
    const int total_violated = groups[0].n_violated + groups[1].n_violated + groups[2].n_violated;
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
      "[CBF t=%lu] DBG u=(%.2f,%.2f,%.2f) dev=%.4f | "
      "R[%d]m=%.3f | P1[%d]m=%.3f | P2[%d]m=%.3f | violated=%d%s",
      tick, u_nom.linear.x, u_nom.linear.y, u_nom.angular.z, cbf_res.qp.deviation,
      groups[0].count, groups[0].worst_margin,
      groups[1].count, groups[1].worst_margin,
      groups[2].count, groups[2].worst_margin,
      total_violated,
      total_violated > 0 && cbf_res.qp.deviation < 1e-3 ? " *** BUG: violated but dev≈0 ***" : "");
    for (int s = 0; s < kMaxSteps; ++s) {
      if (!groups[s].worst) {continue;}
      const auto & c = *groups[s].worst;
      RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
        "  [step=%d worst] cid=%d h=%.4f Au=%.4f b=%.4f margin=%.4f "
        "gx=%.3f gy=%.3f lever=%.3f pos=(%.3f,%.3f) %s",
        s, c.corner_id, c.h, c.Au, c.rhs, c.margin,
        c.gx, c.gy, c.grad.z(), c.sample_x, c.sample_y,
        c.Au > c.rhs + 1e-3 ? "*** VIOLATED ***" : (c.margin < 0.05 ? "TIGHT" : "ok"));
    }

    // Per-constraint topic.
    if (cbf_constraints_pub_ && cbf_constraints_pub_->is_activated()) {
      std_msgs::msg::Float32MultiArray cmsg;
      cmsg.data.reserve(cbf_res.constraints.size() * 12);
      for (const auto & c : cbf_res.constraints) {
        cmsg.data.insert(cmsg.data.end(), {
          static_cast<float>(c.predict_step), static_cast<float>(c.corner_id),
          static_cast<float>(c.h), static_cast<float>(c.Au),
          static_cast<float>(c.margin), static_cast<float>(c.rhs),
          static_cast<float>(c.gx), static_cast<float>(c.gy),
          static_cast<float>(c.grad.z()), static_cast<float>(c.sample_x),
          static_cast<float>(c.sample_y),
          static_cast<float>(c.Au > c.rhs + 1e-3 ? 1.0f : 0.0f)});
      }
      cbf_constraints_pub_->publish(cmsg);
    }

    // Gradient arrow markers.
    if (cbf_gradients_pub_ && cbf_gradients_pub_->is_activated()) {
      visualization_msgs::msg::MarkerArray gma;
      int mid = 0;
      for (const auto & c : cbf_res.constraints) {
        const bool pred = (c.predict_step > 0);
        visualization_msgs::msg::Marker arr;
        arr.header.frame_id = base_frame; arr.header.stamp = stamp;
        arr.ns = pred ? "cbf_grad_pred" : "cbf_grad_reactive"; arr.id = mid++;
        arr.type = visualization_msgs::msg::Marker::ARROW;
        arr.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point ps, pe;
        ps.x = c.sample_x; ps.y = c.sample_y; ps.z = 0.05;
        const double al = pred ? 0.08 : 0.12;
        pe.x = c.sample_x + al*c.gx; pe.y = c.sample_y + al*c.gy; pe.z = 0.05;
        arr.points = {ps, pe};
        arr.scale.x = pred ? 0.015 : 0.025; arr.scale.y = pred ? 0.030 : 0.050; arr.scale.z = 0.0;
        arr.color.a = 1.0f;
        if (c.margin < 0.0)       {arr.color.r=1.0f; arr.color.g=0.0f; arr.color.b=0.0f;}
        else if (c.margin < 0.05) {arr.color.r=1.0f; arr.color.g=0.55f; arr.color.b=0.0f;}
        else                      {arr.color.r=0.0f; arr.color.g=1.0f; arr.color.b=0.3f;}
        if (pred) {arr.color.b = 0.8f;}
        arr.lifetime = rclcpp::Duration::from_seconds(0.3);
        gma.markers.push_back(arr);
      }
      cbf_gradients_pub_->publish(gma);
    }
  }

  // Body-corner RViz markers.
  if (corners_pub_ && corners_pub_->is_activated()) {
    visualization_msgs::msg::MarkerArray ma;
    const auto & corners = cbf_res.corners;
    static const char * const kNames[4] = {"FR","BR","BL","FL"};
    std::array<double,4> h_vals = {1.0,1.0,1.0,1.0};
    for (const auto & c : cbf_res.constraints) {
      if (c.corner_id >= 1 && c.corner_id <= 4)
        h_vals[c.corner_id-1] = std::min(h_vals[c.corner_id-1], c.h);
    }
    for (int i = 0; i < 4; ++i) {
      visualization_msgs::msg::Marker sph;
      sph.header.frame_id = base_frame; sph.header.stamp = stamp;
      sph.ns = "cbf_corners"; sph.id = i;
      sph.type = visualization_msgs::msg::Marker::SPHERE;
      sph.action = visualization_msgs::msg::Marker::ADD;
      sph.pose.position.x = corners[i].x; sph.pose.position.y = corners[i].y;
      sph.pose.position.z = 0.05; sph.pose.orientation.w = 1.0;
      sph.scale.x = sph.scale.y = sph.scale.z = 0.10;
      const double h = h_vals[i];
      sph.color.a = 1.0f;
      if      (h > 0.10) {sph.color.r=0.0f; sph.color.g=1.0f; sph.color.b=0.0f;}
      else if (h > 0.02) {sph.color.r=1.0f; sph.color.g=0.55f; sph.color.b=0.0f;}
      else               {sph.color.r=1.0f; sph.color.g=0.0f; sph.color.b=0.0f;}
      sph.lifetime = rclcpp::Duration::from_seconds(0.3);
      ma.markers.push_back(sph);
      visualization_msgs::msg::Marker txt;
      txt.header = sph.header; txt.ns = "cbf_corner_labels"; txt.id = i;
      txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt.action = visualization_msgs::msg::Marker::ADD;
      txt.pose.position.x = corners[i].x; txt.pose.position.y = corners[i].y;
      txt.pose.position.z = 0.25; txt.pose.orientation.w = 1.0;
      txt.scale.z = 0.08;
      txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0f;
      char buf[32]; std::snprintf(buf, sizeof(buf), "%s\nh=%.3f", kNames[i], h);
      txt.text = buf; txt.lifetime = rclcpp::Duration::from_seconds(0.3);
      ma.markers.push_back(txt);
    }
    corners_pub_->publish(ma);
  }

  // Float32MultiArray for rqt_plot.
  if (debug_pub_ && debug_pub_->is_activated()) {
    std_msgs::msg::Float32MultiArray dbg;
    dbg.data = {
      static_cast<float>(cbf_res.min_h), static_cast<float>(cbf_res.qp.deviation),
      static_cast<float>(cbf_res.constraints.size()),
      static_cast<float>(u_nom.linear.x), static_cast<float>(u_nom.linear.y),
      static_cast<float>(u_nom.angular.z),
      static_cast<float>(out.twist.linear.x), static_cast<float>(out.twist.linear.y),
      static_cast<float>(out.twist.angular.z),
      static_cast<float>(cbf_res.qp.ok ? 0.0f : 1.0f),
      static_cast<float>(yaw_err),
      static_cast<float>(target.dist_to_target)};
    debug_pub_->publish(dbg);
  }

  // ── 10. Position+angle-based stuck detection ──────────────────────────────
  constexpr double kCommandedThresh = 0.05;
  constexpr double kStuckDist       = 0.03;
  constexpr double kStuckAngle      = 0.30;
  constexpr double kStuckTime       = 8.0;
  constexpr double kStuckResetS     = 3.0;

  const double commanded_norm = std::hypot(
    out.twist.linear.x, out.twist.linear.y, out.twist.angular.z);

  if (!odom_path.poses.empty()) {
    if (stuck_start_time_.nanoseconds() == 0) {
      stuck_start_x_   = robot_in_odom.pose.position.x;
      stuck_start_y_   = robot_in_odom.pose.position.y;
      stuck_start_yaw_ = tf2::getYaw(robot_in_odom.pose.orientation);
      stuck_start_time_ = stamp;
    }
    const double moved = std::hypot(
      robot_in_odom.pose.position.x - stuck_start_x_,
      robot_in_odom.pose.position.y - stuck_start_y_);
    const double rotated = std::abs(angles::shortest_angular_distance(
      stuck_start_yaw_, tf2::getYaw(robot_in_odom.pose.orientation)));
    const double elapsed = (stamp - stuck_start_time_).seconds();
    const bool made_progress = (moved > kStuckDist) || (rotated > kStuckAngle);

    if (made_progress) {
      stuck_start_x_   = robot_in_odom.pose.position.x;
      stuck_start_y_   = robot_in_odom.pose.position.y;
      stuck_start_yaw_ = tf2::getYaw(robot_in_odom.pose.orientation);
      stuck_start_time_ = stamp;
      stuck_            = false;
    } else if (elapsed > kStuckTime && commanded_norm > kCommandedThresh && !stuck_) {
      stuck_ = true;
      RCLCPP_WARN(logger_,
        "[CBF t=%lu] *** STUCK *** moved %.3fm rotated %.3frad in %.1fs "
        "while commanding %.3f m/s. h_min=%.3f. Stopping — Nav2 recovery will take over.",
        tick, moved, rotated, elapsed, commanded_norm, cbf_res.min_h);
      log_->event("STUCK — stopping for Nav2 recovery");
    }
  }

  if (stuck_) {
    const double stuck_elapsed = (stamp - stuck_start_time_).seconds();
    if (stuck_elapsed > kStuckResetS) {
      RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
        "[CBF t=%lu] stuck timeout reset — retrying", tick);
      stuck_ = false;
      stuck_start_x_   = robot_in_odom.pose.position.x;
      stuck_start_y_   = robot_in_odom.pose.position.y;
      stuck_start_yaw_ = tf2::getYaw(robot_in_odom.pose.orientation);
      stuck_start_time_ = stamp;
    } else {
      return zeroTwist(base_frame, stamp);
    }
  }
  return out;
}

}  // namespace nav2_constrained_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_constrained_controller::ConstrainedController,
  nav2_core::Controller)
