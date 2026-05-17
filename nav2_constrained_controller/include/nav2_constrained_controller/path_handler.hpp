// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// PathHandler implements Fix 1 from the design doc.
//
//   On setPlan(path_in_map):
//     1. Look up T_odom<-map ONCE (this is the only AMCL touch in the
//        controller's lifetime per plan).
//     2. Reproject every path pose into odom; store as path_in_odom_.
//     3. Run validateOrientations(path_in_odom_) — retangents constant-
//        yaw lattice intermediate poses to atan2(next-this).
//
//   On every subsequent control tick:
//     - getLocalPlan(robot_pose_odom): slice the stored odom-frame path
//       around the robot and transform that slice into base_link using
//       T_base<-odom only. AMCL is NOT touched.
//
// This breaks the per-tick AMCL coupling that the existing graceful
// path_handler had through nav_2d_utils::transformPose(map -> base).

#ifndef NAV2_CONSTRAINED_CONTROLLER__PATH_HANDLER_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__PATH_HANDLER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_constrained_controller
{

class PathHandler
{
public:
  PathHandler(
    tf2::Duration transform_tolerance,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  ~PathHandler() = default;

  // One-shot reprojection of the new global plan into the odom frame.
  // The single AMCL-coupled TF lookup happens here. Returns true on
  // success, false if the lookup or input plan was invalid (caller
  // should preserve previous state).
  bool setPlan(const nav_msgs::msg::Path & path_in_map);

  // Slice the stored odom-frame path around the robot and return the
  // slice in base_link. No AMCL, just T_base<-odom. The slice is
  // bounded by the local costmap dimensions on the far end.
  // Throws nav2_core::PlannerException on TF failure or empty plan.
  nav_msgs::msg::Path getLocalPlan(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double max_robot_pose_search_dist);

  // Returns the cached odom-frame plan (after retangenting). Useful for visualisation.
  const nav_msgs::msg::Path & getPathInOdom() const {return path_in_odom_;}

  // Returns the goal pose in its original map frame so callers can re-transform
  // it using the current AMCL TF — matching how the controller server's
  // isGoalReached() locates the goal. dist_to_goal must use the same TF snapshot
  // as the server, otherwise AMCL drift accumulates into a false distance.
  const geometry_msgs::msg::PoseStamped & getGoalInMap() const {return goal_in_map_;}

  bool hasPlan() const {return !path_in_odom_.poses.empty();}

  // Frame name used by costmap (typically "base_link").
  std::string getBaseFrame() const;

  // Frame name used by the odom-frame plan ("odom" by convention).
  std::string getOdomFrame() const {return odom_frame_;}

protected:
  // Retangent intermediate poses to atan2(next-this). Last pose keeps
  // its goal yaw. Lifted verbatim from nav2_graceful_controller's
  // graceful_controller.cpp:432-447.
  static void validateOrientations(
    std::vector<geometry_msgs::msg::PoseStamped> & poses);

  rclcpp::Duration transform_tolerance_{0, 0};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::string odom_frame_{"odom"};

  // The plan after one-shot reprojection into odom + retangenting.
  nav_msgs::msg::Path path_in_odom_;

  // Goal pose kept in map frame (not converted). Used to re-transform with
  // the current AMCL TF each tick so dist_to_goal tracks AMCL updates.
  geometry_msgs::msg::PoseStamped goal_in_map_;
  rclcpp::Logger logger_{rclcpp::get_logger("ConstrainedPathHandler")};
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__PATH_HANDLER_HPP_
