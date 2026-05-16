// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#include "nav2_constrained_controller/constrained_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "sensor_msgs/point_cloud2_iterator.hpp"
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

  goal_reached_       = false;
  cloud_log_throttle_ = 0;
  esdf_pub_throttle_  = 0;
  has_lidar_tf_       = false;

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
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
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

  // Goal-reached check against the stored odom path.
  const auto & odom_path = path_handler_->getPathInOdom();
  double dist_to_goal = std::numeric_limits<double>::infinity();
  if (!odom_path.poses.empty()) {
    const auto & last = odom_path.poses.back().pose.position;
    geometry_msgs::msg::PoseStamped robot_in_odom;
    try {
      geometry_msgs::msg::TransformStamped tf_odom;
      tf_odom = tf_buffer_->lookupTransform(
        path_handler_->getOdomFrame(), pose.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(pose, robot_in_odom, tf_odom);
      dist_to_goal = std::hypot(
        robot_in_odom.pose.position.x - last.x,
        robot_in_odom.pose.position.y - last.y);
    } catch (const tf2::TransformException &) {}
  }

  if (dist_to_goal < params->goal_dist_tolerance) {
    if (!goal_reached_) {log_->event("goal reached"); goal_reached_ = true;}
    auto stop = zeroTwist(base_frame, stamp);
    log_->logState(tick, stamp.seconds(), 0, 0, 0, stop.twist, stop.twist, false, dist_to_goal);
    return stop;
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
  CbfFilterResult cbf_res;
  cbf_res.u = u_nom;  // pass-through when no ESDF data

  if (esdf_grid_->hasData()) {
    cbf_res = cbf_filter_->filter(*esdf_grid_, u_nom);

    if (!cbf_res.qp.ok) {
      log_->event("QP infeasible — falling back to clamped u_nom");
      cbf_res.u.linear.x  = std::clamp(u_nom.linear.x,  -params->v_linear_max,  params->v_linear_max);
      cbf_res.u.linear.y  = std::clamp(u_nom.linear.y,  -params->v_lateral_max, params->v_lateral_max);
      cbf_res.u.angular.z = std::clamp(u_nom.angular.z, -params->v_angular_max, params->v_angular_max);
    } else {
      // Guard: QP is technically feasible but wz lever-arm at tight h can
      // drive u* to near-zero even when u_nom is large. This happens when
      // opposing wall constraints consume the entire rhs budget through
      // the rotational lever arm. Fall back to u_nom so the robot keeps
      // moving — it is better to move and let the CBF correct next tick
      // than to stop entirely in a narrow passage.
      const double u_star_norm = std::hypot(
        cbf_res.u.linear.x, cbf_res.u.linear.y, cbf_res.u.angular.z);
      const double u_nom_norm = std::hypot(
        u_nom.linear.x, u_nom.linear.y, u_nom.angular.z);
      if (u_star_norm < 0.02 && u_nom_norm > 0.05) {
        log_->event("CBF near-zero output — falling back to clamped u_nom");
        cbf_res.u.linear.x  = std::clamp(u_nom.linear.x,  -params->v_linear_max,  params->v_linear_max);
        cbf_res.u.linear.y  = std::clamp(u_nom.linear.y,  -params->v_lateral_max, params->v_lateral_max);
        cbf_res.u.angular.z = std::clamp(u_nom.angular.z, -params->v_angular_max, params->v_angular_max);
      }
    }
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

  return out;
}

}  // namespace nav2_constrained_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_constrained_controller::ConstrainedController,
  nav2_core::Controller)
