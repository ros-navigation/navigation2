// Copyright (c) 2022 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav2_controller/path_handler.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/controller_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "angles/angles.h"

namespace nav2_controller
{

using nav2_util::geometry_utils::euclidean_distance;

PathHandler::PathHandler(
  Parameters * params,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  params_ = params;
  costmap_ros_ = costmap_ros;
  tf_ = tf;

  if(params_->enforce_path_inversion) {
    inversion_locale_ = 0u;
  }
}

double PathHandler::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_ros_->getCostmap()->getSizeInMetersX(),
    costmap_ros_->getCostmap()->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

void PathHandler::prunePlan(nav_msgs::msg::Path & plan, const PathIterator end)
{
  plan.poses.erase(plan.poses.begin(), end);
}

bool PathHandler::isWithinInversionTolerances(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Keep full path if we are within tolerance of the inversion pose
  const auto last_pose = global_plan_up_to_inversion_.poses.back();
  float distance = hypotf(
    robot_pose.pose.position.x - last_pose.pose.position.x,
    robot_pose.pose.position.y - last_pose.pose.position.y);

  float angle_distance = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation),
    tf2::getYaw(last_pose.pose.orientation));

  return distance <= params_->inversion_xy_tolerance &&
         fabs(angle_distance) <= params_->inversion_yaw_tolerance;
}

void PathHandler::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  global_plan_up_to_inversion_ = global_plan_;
  if(params_->enforce_path_inversion) {
    inversion_locale_ = nav2_util::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
  }
}

geometry_msgs::msg::PoseStamped PathHandler::getTransformedGoal(
  const builtin_interfaces::msg::Time & stamp)
{
  auto goal = global_plan_.poses.back();
  goal.header.frame_id = global_plan_.header.frame_id;
  goal.header.stamp = stamp;
  if (goal.header.frame_id.empty()) {
    throw nav2_core::ControllerTFError("Goal pose has an empty frame_id");
  }
  geometry_msgs::msg::PoseStamped transformed_goal;
  if (!nav2_util::transformPoseInTargetFrame(goal, transformed_goal, *tf_,
      costmap_ros_->getGlobalFrameID(), params_->transform_tolerance))
  {
    throw nav2_core::ControllerTFError("Unable to transform goal pose into costmap frame");
  }
  return transformed_goal;
}

nav_msgs::msg::Path PathHandler::pruneGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_up_to_inversion_.poses.empty()) {
    throw nav2_core::InvalidPath("Received plan with zero length");
  }

  if (params_->interpolate_curvature_after_goal && global_plan_up_to_inversion_.poses.size() == 1) {
    throw nav2_core::InvalidPath("Received plan with length of one");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::transformPoseInTargetFrame(pose, robot_pose, *tf_,
      global_plan_up_to_inversion_.header.frame_id,
      params_->transform_tolerance))
  {
    throw nav2_core::ControllerTFError("Unable to transform robot pose into global plan's frame");
  }

  // Limit the search for the closest pose up to max_robot_pose_search_dist on the path
  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_up_to_inversion_.poses.begin(), global_plan_up_to_inversion_.poses.end(),
      params_->max_robot_pose_search_dist);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto closest_point =
    nav2_util::geometry_utils::min_by(
    global_plan_up_to_inversion_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Make sure we always have at least 2 points on the transformed plan and that we don't prune
  // the global plan below 2 points in order to have always enough point to interpolate the
  // end of path direction
  if (global_plan_up_to_inversion_.poses.begin() != closest_pose_upper_bound &&
    global_plan_up_to_inversion_.poses.size() > 1 &&
    closest_point == std::prev(closest_pose_upper_bound))
  {
    closest_point = std::prev(std::prev(closest_pose_upper_bound));
  }

  auto pruned_plan_end = global_plan_up_to_inversion_.poses.end();
  const double max_costmap_extent = getCostmapMaxExtent();
  //TODO: For RPP and graceful we don't have prune distance, maybe we need to support two kinds of pruned_plan_end here
  // by parameter?
  if (1){
    pruned_plan_end = nav2_util::geometry_utils::first_after_integrated_distance(
    closest_point, global_plan_up_to_inversion_.poses.end(), params_->prune_distance);
  }
  else{
    pruned_plan_end = std::find_if(
    closest_point, global_plan_up_to_inversion_.poses.end(),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(global_plan_pose, robot_pose) > max_costmap_extent;
    });
  }

  nav_msgs::msg::Path pruned_plan;
  pruned_plan.poses.insert(pruned_plan.poses.end(),
                           closest_point, pruned_plan_end);
  pruned_plan.header = global_plan_.header;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  prunePlan(global_plan_up_to_inversion_, closest_point);

  if (params_->enforce_path_inversion && inversion_locale_ != 0u) {
    if (isWithinInversionTolerances(robot_pose)) {
      prunePlan(global_plan_, global_plan_.poses.begin() + inversion_locale_);
      global_plan_up_to_inversion_ = global_plan_;
      inversion_locale_ = nav2_util::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
    }
  }

  if (pruned_plan.poses.empty()) {
    throw nav2_core::InvalidPath("Resulting plan has 0 poses in it.");
  }

  return pruned_plan;
}

}  // namespace nav2_controller
