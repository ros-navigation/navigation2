// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// PathHandler implements Fix 1 from the design doc.
//
//   On setPlan(path_in_map):
//     1. Look up T_odom<-map ONCE (this is the only AMCL touch in the
//        controller's lifetime per plan).
//     2. Reproject every path pose into odom; store as path_in_odom_.
//     The lattice's pose orientations are kept verbatim — for the OMNI
//     motion model they encode intentional body yaws (alley-axis hold,
//     in-place rotations at doors) and must not be retangented.
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

  // Tracker mode for rotate-in-place fidelity (see getTrackingTarget).
  enum TrackMode : int
  {
    TRACK_NORMAL = 0,             // plain lookahead carrot
    TRACK_APPROACH_ROTATION = 1,  // rotation cluster ahead: target clamped to
                                  // the cluster ENTRY pose (pre-rotation yaw)
    TRACK_ROTATE_IN_PLACE = 2,    // at the cluster spot: target = cluster EXIT
                                  // pose (post-rotation yaw, same position)
  };

  // Tracking target for the PD path follower. All in the odom frame, no TF.
  struct TrackingTarget
  {
    geometry_msgs::msg::PoseStamped pose;  // lookahead pose; lattice yaw verbatim
    double dist_to_target{0.0};            // euclidean robot → target
    double cross_track{0.0};               // euclidean robot → closest path pose
                                           // (tracking-quality metric for logs)
    size_t target_idx{0};                  // index into the (pruned) stored path
    size_t n_remaining{0};                 // poses left in the stored path
    int mode{TRACK_NORMAL};                // TrackMode this tick (for logs)
  };

  // Find the closest stored-path pose to the robot (search window bounded by
  // max_robot_pose_search_dist of integrated path length), prune the poses
  // already passed, then walk lookahead_dist metres of arc-length forward and
  // return that pose as the target. Falls back to the final pose when less
  // than lookahead_dist of path remains. robot_in_odom must already be in the
  // odom frame. Returns false if no plan is stored.
  //
  // Rotate-in-place fidelity: the OMNI lattice emits zero-arc-length rotation
  // clusters (consecutive poses, same position, stepping yaw) where rotating
  // anywhere else nearby would collide. Those poses are invisible to a naive
  // arc-length walk, so the target is NOT allowed past a pending cluster:
  //   APPROACH_ROTATION  robot far from cluster → target = entry pose with
  //                      PRE-rotation yaw (drive there, no early rotation;
  //                      yaw_err stays ≈ 0 so the cos taper stays inactive);
  //   ROTATE_IN_PLACE    robot within kRotReachTol of the spot → target =
  //                      exit pose (post-rotation yaw, same position → the
  //                      position PD dies on its own, wz turns the body);
  //   consumed           robot yaw aligned with exit yaw → normal walk
  //                      resumes past the cluster.
  // The engage/done thresholds form a Schmitt trigger (engage stays latched
  // until done) so odom noise cannot flicker the mode.
  bool getTrackingTarget(
    const geometry_msgs::msg::PoseStamped & robot_in_odom,
    double lookahead_dist,
    double max_robot_pose_search_dist,
    TrackingTarget & out);

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
  rclcpp::Duration transform_tolerance_{0, 0};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::string odom_frame_{"odom"};

  // The plan after one-shot reprojection into odom + retangenting.
  nav_msgs::msg::Path path_in_odom_;

  // Rotation-cluster latch (Schmitt trigger): true while the robot is parked
  // at a cluster spot rotating. Engages when within kRotReachTol of a pending
  // cluster; releases only when the robot's yaw aligns with the cluster exit
  // yaw. Reset on setPlan.
  bool rot_engaged_{false};

  // Goal pose kept in map frame (not converted). Used to re-transform with
  // the current AMCL TF each tick so dist_to_goal tracks AMCL updates.
  geometry_msgs::msg::PoseStamped goal_in_map_;
  rclcpp::Logger logger_{rclcpp::get_logger("ConstrainedPathHandler")};
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__PATH_HANDLER_HPP_
