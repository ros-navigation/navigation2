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
  cbf_constraints_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>(
    "constrained_controller/cbf_constraints", rclcpp::SystemDefaultsQoS());
  cbf_gradients_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "constrained_controller/cbf_gradients", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(logger_,
    "ConstrainedController '%s' configured (Stanley+CBF). cloud=%s log=%s",
    plugin_name_.c_str(),
    params->pointcloud_topic.c_str(),
    params->log_dir.c_str());
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

  transformed_plan_pub_->on_activate();
  motion_target_pub_->on_activate();
  esdf_pub_->on_activate();
  corners_pub_->on_activate();
  debug_pub_->on_activate();
  cbf_constraints_pub_->on_activate();
  cbf_gradients_pub_->on_activate();

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
  last_retreat_state_    = 0;
  last_governor_mode_    = 0;
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
  if (cbf_constraints_pub_)  {cbf_constraints_pub_->on_deactivate();}
  if (cbf_gradients_pub_)    {cbf_gradients_pub_->on_deactivate();}
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
  esdf_pub_.reset();
  corners_pub_.reset();
  debug_pub_.reset();
  cbf_constraints_pub_.reset();
  cbf_gradients_pub_.reset();
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
  last_retreat_state_   = 0;
  esdf_grid_->reset();
  cbf_filter_->reset();

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

geometry_msgs::msg::PoseStamped ConstrainedController::pickSafeTarget(
  const nav_msgs::msg::Path & local_plan,
  int * sel_idx_out,
  int * mode_out) const
{
  const auto * params = param_handler_->getParams();
  const double target_dist = params->motion_target_dist;

  // Required ESDF clearance at the body CENTER for the body to fit safely.
  // Body half-extent (worst axis) + d_safe (corner buffer) + extra margin.
  const double half_extent =
    std::max(0.5 * params->footprint_length, params->footprint_db);
  const double d_required = half_extent + params->esdf_d_safe + params->governor_margin;

  // ── Option (a): walk along the local plan for a safe pose at ≥ target_dist
  if (esdf_grid_ && esdf_grid_->hasData()) {
    for (size_t i = 0; i < local_plan.poses.size(); ++i) {
      const auto & p = local_plan.poses[i].pose.position;
      const double d_to_p = std::hypot(p.x, p.y);
      if (d_to_p < target_dist) {continue;}

      const auto q = esdf_grid_->query(p.x, p.y);
      if (!q.valid) {
        // Pose outside the local ESDF grid — accept as-is (CBF will handle).
        if (sel_idx_out) {*sel_idx_out = static_cast<int>(i);}
        if (mode_out)    {*mode_out    = 0;}
        return local_plan.poses[i];
      }
      if (q.d >= d_required) {
        // Safe path pose found. This is the natural governor target.
        if (sel_idx_out) {*sel_idx_out = static_cast<int>(i);}
        if (mode_out)    {*mode_out    = 1;}
        return local_plan.poses[i];
      }
      // Unsafe — keep searching further along the path.
    }
  }

  // ── Option (b): no safe path pose in range. Push the raw lookahead away
  //                from its nearest wall via the ESDF gradient.
  int raw_idx = 0;
  auto raw = getMotionTarget(target_dist, local_plan, &raw_idx);

  if (esdf_grid_ && esdf_grid_->hasData()) {
    const auto q = esdf_grid_->query(raw.pose.position.x, raw.pose.position.y);
    if (q.valid && q.d < d_required) {
      const double step = (d_required - q.d) + 0.05;
      raw.pose.position.x += step * q.gx;
      raw.pose.position.y += step * q.gy;
      if (sel_idx_out) {*sel_idx_out = raw_idx;}
      if (mode_out)    {*mode_out    = 2;}
      return raw;
    }
  }

  // ── No ESDF available, or raw lookahead is already safe — return as-is.
  if (sel_idx_out) {*sel_idx_out = raw_idx;}
  if (mode_out)    {*mode_out    = 0;}
  return raw;
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

  // ── 1. Local plan in base_link (AMCL-free via PathHandler) ───────────────
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

  // ── 2. Lookahead target ───────────────────────────────────────────────────
  // When the reference governor is enabled, it picks a "safe target": either
  // the first path pose with sufficient body-footprint clearance (option a),
  // or the raw lookahead pushed away from the nearest wall via the ESDF
  // gradient (option b). The returned pose becomes BOTH closest and lookahead
  // for Stanley, turning it into a point-tracker for the governor's choice.
  // Falls back to plain pure-pursuit getMotionTarget() when disabled.
  int sel_idx = 0;
  int governor_mode = 0;
  const auto target_ps = params->governor_enabled
    ? pickSafeTarget(local_plan, &sel_idx, &governor_mode)
    : getMotionTarget(params->motion_target_dist, local_plan, &sel_idx);
  if (params->governor_enabled && governor_mode != last_governor_mode_) {
    static const char * const kModeName[3] = {"RAW", "SAFE_PATH(a)", "DEVIATED(b)"};
    log_->event(
      std::string("governor: ") + kModeName[last_governor_mode_] +
      " → " + kModeName[governor_mode] +
      " | target=(" + std::to_string(target_ps.pose.position.x) +
      "," + std::to_string(target_ps.pose.position.y) + ")");
    last_governor_mode_ = governor_mode;
  }
  if (motion_target_pub_ && motion_target_pub_->is_activated()) {
    geometry_msgs::msg::PointStamped pt;
    pt.header = target_ps.header;
    pt.point  = target_ps.pose.position;
    motion_target_pub_->publish(pt);
  }

  // ── 3. dist_to_goal via live AMCL TF ─────────────────────────────────────
  double dist_to_goal = std::numeric_limits<double>::infinity();
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

      dist_to_goal = std::hypot(
        robot_in_odom.pose.position.x - goal_in_odom.pose.position.x,
        robot_in_odom.pose.position.y - goal_in_odom.pose.position.y);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "dist_to_goal TF failed: %s", e.what());
    }
  }

  // ── 4. Near-goal: yaw alignment then stop ────────────────────────────────
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
      log_->logState(tick, stamp.seconds(), 0, 0, 0, stop.twist, stop.twist, false, dist_to_goal);
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

  // ── 5. Stanley nominal controller ────────────────────────────────────────
  // With governor ON, treat Stanley as a point-tracker for the safe target:
  // pass target_ps as both `closest` and `lookahead` so vx/vy/wz all aim at
  // the governor's choice instead of mixing the raw closest pose's tangent
  // with the (possibly deviated) lookahead. With governor OFF, keep the
  // original Stanley semantics (closest = local_plan.poses[0]).
  NominalDebug nom_dbg;
  const auto & stanley_closest =
    params->governor_enabled ? target_ps.pose : local_plan.poses[0].pose;
  auto u_nom = nominal_->compute(
    stanley_closest, target_ps.pose, dist_to_goal, &nom_dbg);

  // ── 5b. Goal-aware bias: shift QP target toward a far path waypoint ──────
  // Adds λ·g to u_nom before handing to the QP. g is the unit direction in
  // base_link from the robot (origin) to a pose at goal_bias_lookahead_dist
  // arc-length along the local plan. Equivalent to a linear -λ·(g·u) term
  // in the QP objective: the projection lands further along g than a pure
  // ‖u-u_nom‖² projection would have (rubber-band pull toward the path).
  // Translation only (no angular bias). λ=0 reproduces original behaviour.
  geometry_msgs::msg::Twist u_target = u_nom;
  double bias_x = 0.0, bias_y = 0.0;
  if (params->goal_bias_weight > 0.0 && local_plan.poses.size() >= 2) {
    const double L_bias = params->goal_bias_lookahead_dist;
    double cum = 0.0;
    size_t idx = local_plan.poses.size() - 1;  // fallback: end of slice
    for (size_t i = 1; i < local_plan.poses.size(); ++i) {
      const auto & a = local_plan.poses[i - 1].pose.position;
      const auto & b = local_plan.poses[i].pose.position;
      cum += std::hypot(b.x - a.x, b.y - a.y);
      if (cum >= L_bias) {idx = i; break;}
    }
    const auto & far = local_plan.poses[idx].pose.position;
    const double norm = std::hypot(far.x, far.y);
    if (norm > 1e-3) {
      bias_x = params->goal_bias_weight * (far.x / norm);
      bias_y = params->goal_bias_weight * (far.y / norm);
      u_target.linear.x += bias_x;
      u_target.linear.y += bias_y;
    }
  }

  // ── 6. ESDF update ────────────────────────────────────────────────────────
  {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    {std::lock_guard<std::mutex> lock(cloud_mutex_); cloud = last_cloud_;}
    if (cloud) {
      if (!updateEsdf(*cloud)) {log_->event("updateEsdf failed (TF)");}
    } else {
      log_->event("no point cloud yet — CBF pass-through");
    }
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

  // ── 7. CBF safety filter ──────────────────────────────────────────────────
  CbfFilterResult cbf_res;
  cbf_res.u = u_target;
  if (esdf_grid_->hasData()) {
    cbf_res = cbf_filter_->filter(*esdf_grid_, u_target);
    if (!cbf_res.qp.ok) {
      log_->event("QP box-infeasible — zeroing for safety");
      cbf_res.u.linear.x = cbf_res.u.linear.y = cbf_res.u.angular.z = 0.0;
    }
    if (cbf_res.qp.max_slack > 0.01) {
      log_->event("CBF slack: " + std::to_string(cbf_res.qp.max_slack) + " m/s");
    }
    // Log retreat-overlay state transitions exactly once each. picked_candidate
    // is 1..9 inside RETREAT, 0 elsewhere. picked_score is the predicted min_h
    // of the chosen candidate; runner-up info isn't carried but the QP log's
    // deviation will reflect how much we deviated from u_nom toward retreat.
    const int rs_now = static_cast<int>(cbf_res.retreat_state);
    if (rs_now != last_retreat_state_) {
      static const char * const kNames[2] = {"NORMAL", "RETREAT"};
      log_->event(
        std::string("retreat: ") + kNames[last_retreat_state_] + " → " +
        kNames[rs_now] +
        " | min_h_react=" + std::to_string(cbf_res.min_h_react) +
        " | candidate=" + std::to_string(cbf_res.picked_candidate) +
        " | pred_min_h=" + std::to_string(cbf_res.picked_score));
      last_retreat_state_ = rs_now;
    }
    last_min_h_ = cbf_res.min_h;
  }

  // ── 8. Assemble output ────────────────────────────────────────────────────
  geometry_msgs::msg::TwistStamped out;
  out.header.frame_id = base_frame;
  out.header.stamp    = stamp;
  out.twist           = cbf_res.u;

  log_->logState(tick, stamp.seconds(), 0, 0, 0,
    u_nom, out.twist, !nom_dbg.is_forward, dist_to_goal);
  log_->logPath(tick, stamp.seconds(),
    target_ps.pose.position.x, target_ps.pose.position.y,
    tf2::getYaw(target_ps.pose.orientation),
    nom_dbg.r, nom_dbg.s, nom_dbg.ramp, nom_dbg.heading_err,
    sel_idx, static_cast<int>(local_plan.poses.size()));
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
    "[CBF t=%lu] %s cte=%.3fm herr=%.3fr gov=%d | "
    "u_nom=(%.2f,%.2f,%.2f) bias=(%.2f,%.2f) u*=(%.2f,%.2f,%.2f) | "
    "h_min=%.3f cst=%zu dev=%.3f slack=%.4f | dist=%.2fm",
    tick, nom_dbg.is_forward ? "FWD" : "BWD",
    nom_dbg.cte_y, nom_dbg.heading_err, governor_mode,
    u_nom.linear.x, u_nom.linear.y, u_nom.angular.z,
    bias_x, bias_y,
    out.twist.linear.x, out.twist.linear.y, out.twist.angular.z,
    cbf_res.min_h, cbf_res.constraints.size(),
    cbf_res.qp.deviation, cbf_res.qp.max_slack, dist_to_goal);

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
      static_cast<float>(nom_dbg.is_forward ? 0.0f : 1.0f)};
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
