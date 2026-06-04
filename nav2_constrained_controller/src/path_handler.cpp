// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#include "nav2_constrained_controller/path_handler.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <vector>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_core/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "angles/angles.h"

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

  // Lattice yaws (OMNI motion model) carry intent — keep them verbatim.
  path_in_odom_ = std::move(reprojected);
  rot_engaged_ = false;  // fresh plan → fresh rotation-cluster latch

  // Keep goal in original map frame so dist_to_goal can be recomputed each
  // tick using the current AMCL TF — matching the controller server's
  // isGoalReached() which also re-transforms from map using the live TF.
  goal_in_map_ = path_in_map.poses.back();
  goal_in_map_.header.frame_id = source_frame;

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

bool PathHandler::getTrackingTarget(
  const geometry_msgs::msg::PoseStamped & robot_in_odom,
  double lookahead_dist,
  double max_robot_pose_search_dist,
  TrackingTarget & out)
{
  if (path_in_odom_.poses.empty()) {
    return false;
  }

  // Closest pose within the search window (same scheme as getLocalPlan).
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

  // Cross-track error: robot's distance to the closest path pose. This is
  // the primary tracking-quality metric (0 = perfectly on the lattice path).
  out.cross_track = euclidean_distance(robot_in_odom, *closest_it);

  // Prune passed poses so the next tick's search starts at the robot.
  path_in_odom_.poses.erase(path_in_odom_.poses.begin(), closest_it);

  // ── Rotate-in-place clusters (see header doc) ─────────────────────────────
  // Cluster = consecutive poses < kRotPosEps apart with > kRotYawEps yaw step.
  // A pending cluster within the carrot's reach clamps the target:
  // entry pose (pre-yaw) while approaching, exit pose (post-yaw) once there.
  constexpr double kRotPosEps     = 0.02;  // same-position threshold (m)
  constexpr double kRotYawEps     = 0.05;  // yaw-step threshold (rad)
  constexpr double kRotReachTol   = 0.15;  // engage radius at the spot (m)
  constexpr double kRotDoneTol    = 0.08;  // exit-yaw alignment → done (rad)
  constexpr double kRotPendingTol = 0.20;  // re-trigger needs err above this
                                           // (Schmitt: done < pending)

  const double robot_yaw = tf2::getYaw(robot_in_odom.pose.orientation);
  {
    double walked = 0.0;
    const auto & poses = path_in_odom_.poses;
    for (size_t i = 0; i + 1 < poses.size(); ++i) {
      const double step = euclidean_distance(poses[i], poses[i + 1]);
      const double dyaw = std::abs(angles::shortest_angular_distance(
        tf2::getYaw(poses[i].pose.orientation),
        tf2::getYaw(poses[i + 1].pose.orientation)));

      if (step < kRotPosEps && dyaw > kRotYawEps) {
        // Cluster entry at i — extend to the last same-position pose.
        size_t e = i + 1;
        while (e + 1 < poses.size() &&
          euclidean_distance(poses[e], poses[e + 1]) < kRotPosEps)
        {
          ++e;
        }
        const double exit_yaw = tf2::getYaw(poses[e].pose.orientation);
        const double yaw_to_exit = std::abs(
          angles::shortest_angular_distance(robot_yaw, exit_yaw));
        const bool pending = rot_engaged_ ?
          (yaw_to_exit > kRotDoneTol) : (yaw_to_exit > kRotPendingTol);

        if (!pending) {
          // Cluster consumed (robot aligned) — release latch, scan past it.
          rot_engaged_ = false;
          i = e;
          continue;
        }

        const double dist_to_spot = euclidean_distance(robot_in_odom, poses[i]);
        if (!rot_engaged_ && dist_to_spot < kRotReachTol) {
          rot_engaged_ = true;  // latched until yaw aligns (Schmitt)
        }

        const size_t idx = rot_engaged_ ? e : i;
        out.pose = poses[idx];
        out.dist_to_target = euclidean_distance(robot_in_odom, poses[idx]);
        out.target_idx = idx;
        out.n_remaining = poses.size();
        out.mode = rot_engaged_ ? TRACK_ROTATE_IN_PLACE : TRACK_APPROACH_ROTATION;
        return true;
      }

      walked += step;
      if (walked > lookahead_dist) {break;}  // clusters beyond the carrot can wait
    }
  }

  // Walk lookahead_dist of integrated arc-length from the closest pose;
  // saturate at the final pose (goal) when the path is shorter.
  auto target_it =
    nav2_util::geometry_utils::first_after_integrated_distance(
    path_in_odom_.poses.begin(), path_in_odom_.poses.end(), lookahead_dist);
  if (target_it == path_in_odom_.poses.end()) {
    target_it = std::prev(path_in_odom_.poses.end());
  }

  out.pose = *target_it;
  out.dist_to_target = euclidean_distance(robot_in_odom, *target_it);
  out.target_idx = static_cast<size_t>(
    std::distance(path_in_odom_.poses.begin(), target_it));
  out.n_remaining = path_in_odom_.poses.size();
  out.mode = TRACK_NORMAL;
  return true;
}

}  // namespace nav2_constrained_controller
