// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#include "nav2_constrained_controller/constrained_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
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
  if (!node) {
    throw std::runtime_error("ConstrainedController: invalid parent node");
  }

  plugin_name_ = name;
  tf_buffer_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  param_handler_ = std::make_unique<ParameterHandler>(
    parent, plugin_name_, logger_);
  auto * params = param_handler_->getParams();

  // Use the costmap's transform tolerance default (matches what other
  // Nav2 controllers do). 0.1s is a safe baseline.
  const tf2::Duration transform_tol = tf2::durationFromSec(0.1);

  path_handler_ = std::make_unique<PathHandler>(
    transform_tol, tf_buffer_, costmap_ros_);
  nominal_ = std::make_unique<NominalController>(params);
  scene_parser_ = std::make_unique<SceneParser>(params);
  cbf_filter_ = std::make_unique<CBFSafetyFilter>(params);
  log_ = std::make_unique<Logger>();

  transformed_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "constrained_controller/transformed_local_plan", 1);
  motion_target_pub_ =
    node->create_publisher<geometry_msgs::msg::PointStamped>(
    "constrained_controller/motion_target", 1);

  RCLCPP_INFO(
    logger_,
    "ConstrainedController '%s' configured. log_dir=%s, lidar=%s",
    plugin_name_.c_str(),
    params->log_dir.c_str(),
    params->lidar_topic.c_str());
}

void ConstrainedController::activate()
{
  auto node = parent_.lock();
  if (!node) {return;}
  auto * params = param_handler_->getParams();

  log_->open(params->log_dir, params->log_enabled);
  log_->event("activate");

  // Subscribe to LiDAR. Best-effort QoS to match typical sensor pubs.
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  lidar_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    params->lidar_topic, qos,
    std::bind(
      &ConstrainedController::lidarCallback, this,
      std::placeholders::_1));

  transformed_plan_pub_->on_activate();
  motion_target_pub_->on_activate();

  goal_reached_ = false;
  lidar_log_throttle_ = 0;

  RCLCPP_INFO(logger_, "ConstrainedController activated.");
}

void ConstrainedController::deactivate()
{
  if (log_) {log_->event("deactivate"); log_->close();}
  lidar_sub_.reset();
  if (transformed_plan_pub_) {transformed_plan_pub_->on_deactivate();}
  if (motion_target_pub_) {motion_target_pub_->on_deactivate();}
}

void ConstrainedController::cleanup()
{
  param_handler_.reset();
  path_handler_.reset();
  nominal_.reset();
  scene_parser_.reset();
  cbf_filter_.reset();
  log_.reset();
  transformed_plan_pub_.reset();
  motion_target_pub_.reset();
}

void ConstrainedController::setPlan(const nav_msgs::msg::Path & path)
{
  // Fix 1 lives entirely inside path_handler_->setPlan: single AMCL
  // touch, reproject, retangent. Errors are logged but do not throw —
  // Nav2 will detect missing plan via getLocalPlan exceptions later.
  const bool ok = path_handler_->setPlan(path);
  goal_reached_ = false;
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
    p->v_linear_max = std::max(
      p->v_linear_max_initial * speed_limit / 100.0,
      p->v_linear_min);
    p->v_angular_max = p->v_angular_max_initial * speed_limit / 100.0;
  } else {
    p->v_linear_max = std::max(speed_limit, p->v_linear_min);
    p->v_angular_max = p->v_angular_max_initial *
      speed_limit / std::max(1e-6, p->v_linear_max_initial);
  }
}

void ConstrainedController::lidarCallback(
  sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  last_scan_ = msg;
}

bool ConstrainedController::scanToBaseLink(
  const sensor_msgs::msg::LaserScan & scan,
  std::vector<Point2D> & out_points,
  const rclcpp::Time & stamp) const
{
  out_points.clear();
  out_points.reserve(scan.ranges.size());

  const std::string base_frame = costmap_ros_->getBaseFrameID();
  geometry_msgs::msg::TransformStamped tf_base_from_sensor;
  try {
    tf_base_from_sensor = tf_buffer_->lookupTransform(
      base_frame, scan.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "scanToBaseLink: TF %s<-%s failed: %s",
      base_frame.c_str(), scan.header.frame_id.c_str(), e.what());
    (void)stamp;
    return false;
  }

  const auto * params = param_handler_->getParams();
  const double r_min = std::max(
    static_cast<double>(scan.range_min), params->lidar_min_range);
  const double r_max = std::min(
    static_cast<double>(scan.range_max), params->lidar_max_range);

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r < r_min || r > r_max) {continue;}
    const double angle = scan.angle_min + i * scan.angle_increment;
    geometry_msgs::msg::PointStamped p_in;
    p_in.header = scan.header;
    p_in.point.x = r * std::cos(angle);
    p_in.point.y = r * std::sin(angle);
    p_in.point.z = 0.0;
    geometry_msgs::msg::PointStamped p_out;
    tf2::doTransform(p_in, p_out, tf_base_from_sensor);
    out_points.push_back({p_out.point.x, p_out.point.y});
  }
  return true;
}

geometry_msgs::msg::PoseStamped ConstrainedController::getMotionTarget(
  double motion_target_dist,
  const nav_msgs::msg::Path & local_plan,
  int * sel_idx_out) const
{
  // Pure-pursuit by L2 distance from base_link origin. Identical to
  // graceful's getMotionTarget, lifted with one extra side output.
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
  t.header.stamp = stamp;
  return t;
}

geometry_msgs::msg::TwistStamped
ConstrainedController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> param_lock(param_handler_->getMutex());
  const auto * params = param_handler_->getParams();
  const std::string base_frame = costmap_ros_->getBaseFrameID();
  const rclcpp::Time stamp = pose.header.stamp;

  const uint64_t tick = log_->newTick(stamp.seconds());

  // 1. Slice the stored odom-frame plan into base_link.
  nav_msgs::msg::Path local_plan;
  try {
    local_plan = path_handler_->getLocalPlan(
      pose, params->max_robot_pose_search_dist);
  } catch (const std::exception & e) {
    log_->event(std::string("getLocalPlan failed: ") + e.what());
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "computeVelocityCommands: %s", e.what());
    return zeroTwist(base_frame, stamp);
  }

  if (transformed_plan_pub_ && transformed_plan_pub_->is_activated()) {
    transformed_plan_pub_->publish(local_plan);
  }

  // 2. Lookahead.
  int sel_idx = 0;
  const auto target_ps = getMotionTarget(
    params->motion_target_dist, local_plan, &sel_idx);

  if (motion_target_pub_ && motion_target_pub_->is_activated()) {
    geometry_msgs::msg::PointStamped pt;
    pt.header = target_ps.header;
    pt.point = target_ps.pose.position;
    motion_target_pub_->publish(pt);
  }

  // Goal-reached check: distance to last pose of the stored odom path
  // (after pruning the local slice already advanced it).
  const auto & odom_path = path_handler_->getPathInOdom();
  double dist_to_goal = std::numeric_limits<double>::infinity();
  if (!odom_path.poses.empty()) {
    const auto & last = odom_path.poses.back().pose.position;
    geometry_msgs::msg::PoseStamped robot_in_odom;
    geometry_msgs::msg::TransformStamped tf_odom_from_pose;
    try {
      tf_odom_from_pose = tf_buffer_->lookupTransform(
        path_handler_->getOdomFrame(), pose.header.frame_id,
        tf2::TimePointZero);
      tf2::doTransform(pose, robot_in_odom, tf_odom_from_pose);
      dist_to_goal = std::hypot(
        robot_in_odom.pose.position.x - last.x,
        robot_in_odom.pose.position.y - last.y);
    } catch (const tf2::TransformException &) {
      // Non-fatal — leave dist_to_goal at infinity.
    }
  }
  if (dist_to_goal < params->goal_dist_tolerance) {
    if (!goal_reached_) {
      log_->event("goal reached");
      goal_reached_ = true;
    }
    auto stop = zeroTwist(base_frame, stamp);
    log_->logState(
      tick, stamp.seconds(),
      0.0, 0.0, 0.0,
      stop.twist, stop.twist, false, dist_to_goal);
    return stop;
  }

  // 3. Nominal P controller.
  NominalDebug nom_dbg;
  const auto u_nom = nominal_->compute(target_ps.pose, &nom_dbg);

  // 4. LiDAR scene parse.
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan = last_scan_;
  }

  std::vector<Point2D> points_base;
  SceneSnapshot snap;
  snap.stamp_sec = stamp.seconds();
  if (scan) {
    if (scanToBaseLink(*scan, points_base, stamp)) {
      snap = scene_parser_->parse(points_base, stamp.seconds());
    } else {
      log_->event("scanToBaseLink failed (TF lookup)");
    }
  } else {
    log_->event("no LiDAR scan available yet — skipping CBF this tick");
  }

  // 5. CBF safety filter.
  CbfFilterResult cbf_res;
  cbf_res.u = u_nom;  // pass-through fallback
  if (!snap.walls.empty()) {
    cbf_res = cbf_filter_->filter(snap, u_nom);
  }

  // 6. Logging.
  geometry_msgs::msg::TwistStamped out;
  out.header.frame_id = base_frame;
  out.header.stamp = stamp;
  out.twist = cbf_res.u;

  log_->logState(
    tick, stamp.seconds(),
    0.0, 0.0, 0.0,  // rx,ry,ryaw (we operate in base_link)
    u_nom, out.twist,
    u_nom.linear.x < 0.0,
    dist_to_goal);

  log_->logPath(
    tick, stamp.seconds(),
    target_ps.pose.position.x,
    target_ps.pose.position.y,
    tf2::getYaw(target_ps.pose.orientation),
    nom_dbg.r, nom_dbg.s, nom_dbg.ramp, nom_dbg.yaw_err,
    sel_idx, static_cast<int>(local_plan.poses.size()));

  log_->logWalls(tick, stamp.seconds(), snap.walls);
  log_->logCorners(tick, stamp.seconds(), snap.corners);
  log_->logPassage(tick, stamp.seconds(), snap.passage);

  if (!cbf_res.constraints.empty()) {
    const Eigen::Vector3d u_nom_e(
      u_nom.linear.x, u_nom.linear.y, u_nom.angular.z);
    const Eigen::Vector3d u_final_e(
      out.twist.linear.x, out.twist.linear.y, out.twist.angular.z);
    log_->logCbfConstraints(
      tick, stamp.seconds(), cbf_res.constraints, u_nom_e, u_final_e);
    log_->logQp(
      tick, stamp.seconds(),
      cbf_res.qp.ok,
      static_cast<int>(cbf_res.constraints.size()),
      cbf_res.qp.n_active,
      cbf_res.solve_time_us,
      cbf_res.qp.deviation,
      cbf_res.qp.iterations);
  }

  // Throttled raw LiDAR log.
  if (scan && params->log_lidar_every_n_ticks > 0) {
    if (++lidar_log_throttle_ %
      static_cast<uint64_t>(params->log_lidar_every_n_ticks) == 0)
    {
      log_->logLidar(tick, stamp.seconds(), *scan);
    }
  }

  return out;
}

}  // namespace nav2_constrained_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_constrained_controller::ConstrainedController,
  nav2_core::Controller)
