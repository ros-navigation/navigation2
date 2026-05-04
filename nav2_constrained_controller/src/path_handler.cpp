// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#include "nav2_constrained_controller/path_handler.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_core/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_constrained_controller
{

using nav2_util::geometry_utils::euclidean_distance;

PathHandler::PathHandler(
  tf2::Duration transform_tolerance,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: transform_tolerance_(transform_tolerance),
  tf_buffer_(tf),
  costmap_ros_(costmap_ros)
{
  // Use the costmap's global frame as the "odom" frame here. In the
  // canonical Nav2 setup this IS odom, but some configurations bind
  // the local costmap to a different frame and we follow that lead.
  odom_frame_ = costmap_ros_->getGlobalFrameID();
}

std::string PathHandler::getBaseFrame() const
{
  return costmap_ros_->getBaseFrameID();
}

void PathHandler::validateOrientations(
  std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  // Lifted verbatim from nav2_graceful_controller graceful_controller.cpp:432-447.
  // SmacLattice intermediate poses can carry a constant yaw across
  // position-changing waypoints (the lattice was built diff-drive-style
  // with arc primitives). Without retangenting, the heading-error P
  // loop gets a lookahead yaw that snaps when the rolling horizon
  // crosses a path bend, producing phantom rotations.
  if (poses.size() < 3) {
    return;
  }
  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    const double dx = poses[i + 1].pose.position.x - poses[i].pose.position.x;
    const double dy = poses[i + 1].pose.position.y - poses[i].pose.position.y;
    if (std::hypot(dx, dy) < 1e-9) {
      continue;  // duplicate poses -> keep prior orientation
    }
    const double yaw = std::atan2(dy, dx);
    poses[i].pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  }
}

bool PathHandler::setPlan(const nav_msgs::msg::Path & path_in_map)
{
  if (path_in_map.poses.empty()) {
    RCLCPP_WARN(logger_, "setPlan: received empty plan, ignoring");
    return false;
  }

  // The one and only AMCL-coupled TF lookup per plan. We pull
  // T_odom<-map at the planner's stamp so the rest of the plan's
  // lifetime is map-free.
  const std::string source_frame = path_in_map.header.frame_id;
  const std::string target_frame = odom_frame_;

  geometry_msgs::msg::TransformStamped tf_odom_from_map;
  try {
    // Use latest-available TF; per-plan AMCL freshness is what we
    // want, not a strict timestamp match.
    tf_odom_from_map = tf_buffer_->lookupTransform(
      target_frame, source_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(
      logger_, "setPlan: failed to look up T_%s<-%s: %s",
      target_frame.c_str(), source_frame.c_str(), e.what());
    return false;
  }

  nav_msgs::msg::Path reprojected;
  reprojected.header = path_in_map.header;
  reprojected.header.frame_id = target_frame;
  reprojected.poses.reserve(path_in_map.poses.size());

  for (const auto & ps_map : path_in_map.poses) {
    geometry_msgs::msg::PoseStamped ps_odom;
    ps_odom.header = ps_map.header;
    ps_odom.header.frame_id = target_frame;
    tf2::doTransform(ps_map, ps_odom, tf_odom_from_map);
    reprojected.poses.push_back(ps_odom);
  }

  validateOrientations(reprojected.poses);
  path_in_odom_ = std::move(reprojected);

  RCLCPP_INFO(
    logger_,
    "setPlan: reprojected %zu poses from '%s' to '%s' (single AMCL touch)",
    path_in_odom_.poses.size(),
    source_frame.c_str(), target_frame.c_str());
  return true;
}

nav_msgs::msg::Path PathHandler::getLocalPlan(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double max_robot_pose_search_dist)
{
  if (path_in_odom_.poses.empty()) {
    throw nav2_core::PlannerException(
            "ConstrainedPathHandler::getLocalPlan: no plan stored");
  }

  // Robot pose may arrive in any frame; bring it into odom (no AMCL —
  // T_odom<-base comes from the wheel/IMU EKF).
  geometry_msgs::msg::PoseStamped robot_in_odom;
  if (!nav_2d_utils::transformPose(
      tf_buffer_, odom_frame_, robot_pose, robot_in_odom,
      transform_tolerance_))
  {
    throw nav2_core::PlannerException(
            "ConstrainedPathHandler::getLocalPlan: failed to transform robot "
            "pose into odom frame");
  }

  // Find the closest pose on the stored path within a search window.
  auto search_end =
    nav2_util::geometry_utils::first_after_integrated_distance(
    path_in_odom_.poses.begin(), path_in_odom_.poses.end(),
    max_robot_pose_search_dist);

  auto closest_it =
    nav2_util::geometry_utils::min_by(
    path_in_odom_.poses.begin(), search_end,
    [&robot_in_odom](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_in_odom, ps);
    });

  // Far end of slice: bounded by half the local costmap span (so the
  // lookahead picker never falls off the costmap).
  const double dist_threshold = std::max(
    costmap_ros_->getCostmap()->getSizeInMetersX(),
    costmap_ros_->getCostmap()->getSizeInMetersY()) / 2.0;
  auto slice_end = std::find_if(
    closest_it, path_in_odom_.poses.end(),
    [&](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(ps, robot_in_odom) > dist_threshold;
    });

  // Transform the slice from odom -> base_link via T_base<-odom only.
  // This is the AMCL-free per-tick lookup.
  const std::string base_frame = costmap_ros_->getBaseFrameID();
  geometry_msgs::msg::TransformStamped tf_base_from_odom;
  try {
    tf_base_from_odom = tf_buffer_->lookupTransform(
      base_frame, odom_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    throw nav2_core::PlannerException(
            std::string("ConstrainedPathHandler::getLocalPlan: T_base<-odom "
            "lookup failed: ") + e.what());
  }

  nav_msgs::msg::Path local;
  local.header.frame_id = base_frame;
  local.header.stamp = robot_pose.header.stamp;
  local.poses.reserve(static_cast<size_t>(std::distance(closest_it, slice_end)));
  for (auto it = closest_it; it != slice_end; ++it) {
    geometry_msgs::msg::PoseStamped p_base;
    p_base.header.frame_id = base_frame;
    p_base.header.stamp = robot_pose.header.stamp;
    tf2::doTransform(*it, p_base, tf_base_from_odom);
    p_base.pose.position.z = 0.0;
    local.poses.push_back(p_base);
  }

  // Path pruning: drop already-passed poses from the stored plan so we
  // do not re-search them next tick.
  path_in_odom_.poses.erase(path_in_odom_.poses.begin(), closest_it);

  if (local.poses.empty()) {
    throw nav2_core::PlannerException(
            "ConstrainedPathHandler::getLocalPlan: empty slice "
            "(robot may have overshot path)");
  }
  return local;
}

}  // namespace nav2_constrained_controller
