// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Dexory
// Copyright (c) 2023 Open Navigation LLC
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

#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/robot_utils.hpp"

namespace mppi
{

void PathHandler::initialize(
  nav2::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
  std::shared_ptr<tf2_ros::Buffer> buffer, ParametersHandler * param_handler)
{
  name_ = name;
  costmap_ = costmap;
  tf_buffer_ = buffer;
  auto node = parent.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(max_robot_pose_search_dist_, "max_robot_pose_search_dist", getMaxCostmapDist());
  getParam(prune_distance_, "prune_distance", 1.5);
  getParam(transform_tolerance_, "transform_tolerance", 0.1);
  getParam(enforce_path_inversion_, "enforce_path_inversion", false);
  if (enforce_path_inversion_) {
    getParam(inversion_xy_tolerance_, "inversion_xy_tolerance", 0.2);
    getParam(inversion_yaw_tolerance, "inversion_yaw_tolerance", 0.4);
    inversion_locale_ = 0u;
  }
}

std::pair<nav_msgs::msg::Path, PathIterator>
PathHandler::getGlobalPlanConsideringBoundsInCostmapFrame(
  const geometry_msgs::msg::PoseStamped & global_pose)
{
  using nav2_util::geometry_utils::euclidean_distance;

  auto begin = global_plan_up_to_inversion_.poses.begin();

  // Limit the search for the closest pose up to max_robot_pose_search_dist on the path
  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_up_to_inversion_.poses.begin(), global_plan_up_to_inversion_.poses.end(),
    max_robot_pose_search_dist_);

  // Find closest point to the robot
  auto closest_point = nav2_util::geometry_utils::min_by(
    begin, closest_pose_upper_bound,
    [&global_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(global_pose, ps);
    });

  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_->getGlobalFrameID();
  transformed_plan.header.stamp = global_pose.header.stamp;

  auto pruned_plan_end =
    nav2_util::geometry_utils::first_after_integrated_distance(
    closest_point, global_plan_up_to_inversion_.poses.end(), prune_distance_);

  unsigned int mx, my;
  // Find the furthest relevant pose on the path to consider within costmap
  // bounds
  // Transforming it to the costmap frame in the same loop
  for (auto global_plan_pose = closest_point; global_plan_pose != pruned_plan_end;
    ++global_plan_pose)
  {
    // Transform from global plan frame to costmap frame
    geometry_msgs::msg::PoseStamped costmap_plan_pose;
    global_plan_pose->header.stamp = global_pose.header.stamp;
    global_plan_pose->header.frame_id = global_plan_.header.frame_id;
    nav2_util::transformPoseInTargetFrame(*global_plan_pose, costmap_plan_pose, *tf_buffer_,
        costmap_->getGlobalFrameID(), transform_tolerance_);

    // Check if pose is inside the costmap
    if (!costmap_->getCostmap()->worldToMap(
        costmap_plan_pose.pose.position.x, costmap_plan_pose.pose.position.y, mx, my))
    {
      return {transformed_plan, closest_point};
    }

    // Filling the transformed plan to return with the transformed pose
    transformed_plan.poses.push_back(costmap_plan_pose);
  }

  return {transformed_plan, closest_point};
}

geometry_msgs::msg::PoseStamped PathHandler::transformToGlobalPlanFrame(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_up_to_inversion_.poses.empty()) {
    throw nav2_core::InvalidPath("Received plan with zero length");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::transformPoseInTargetFrame(pose, robot_pose, *tf_buffer_,
      global_plan_up_to_inversion_.header.frame_id, transform_tolerance_))
  {
    throw nav2_core::ControllerTFError(
            "Unable to transform robot pose into global plan's frame");
  }

  return robot_pose;
}

nav_msgs::msg::Path PathHandler::transformPath(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Find relevant bounds of path to use
  geometry_msgs::msg::PoseStamped global_pose =
    transformToGlobalPlanFrame(robot_pose);
  auto [transformed_plan, lower_bound] = getGlobalPlanConsideringBoundsInCostmapFrame(global_pose);

  prunePlan(global_plan_up_to_inversion_, lower_bound);

  if (enforce_path_inversion_ && inversion_locale_ != 0u) {
    if (isWithinInversionTolerances(global_pose)) {
      prunePlan(global_plan_, global_plan_.poses.begin() + inversion_locale_);
      global_plan_up_to_inversion_ = global_plan_;
      inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
    }
  }

  // Calculate distances using the original global plan
  // Find the distance from lower_bound (closest point to robot) to the end of global_plan_up_to_inversion_
  auto start_idx = std::distance(global_plan_up_to_inversion_.poses.begin(), lower_bound);
  global_plan_length_up_to_inversion_ = nav2_util::geometry_utils::calculate_path_length(global_plan_up_to_inversion_, start_idx);
  
  if (enforce_path_inversion_ && inversion_locale_ != 0u) {
    // For paths with inversion: length from closest point to robot to end of full path
    auto full_plan_start_idx = std::distance(global_plan_.poses.begin(), 
      global_plan_.poses.begin() + std::distance(global_plan_up_to_inversion_.poses.begin(), lower_bound));
    global_plan_length_ = nav2_util::geometry_utils::calculate_path_length(global_plan_, full_plan_start_idx);
  }
  else {
    global_plan_length_ = global_plan_length_up_to_inversion_;
  }

  if (transformed_plan.poses.empty()) {
    throw nav2_core::InvalidPath("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

double PathHandler::getMaxCostmapDist()
{
  const auto & costmap = costmap_->getCostmap();
  return static_cast<double>(std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY())) *
         costmap->getResolution() * 0.50;
}

void PathHandler::setPath(const nav_msgs::msg::Path & plan)
{
  global_plan_ = plan;
  global_plan_up_to_inversion_ = global_plan_;
  if (enforce_path_inversion_) {
    inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
  }
}

nav_msgs::msg::Path & PathHandler::getPath() {return global_plan_;}

void PathHandler::prunePlan(nav_msgs::msg::Path & plan, const PathIterator end)
{
  plan.poses.erase(plan.poses.begin(), end);
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
  if (!nav2_util::transformPoseInTargetFrame(goal, transformed_goal, *tf_buffer_,
      costmap_->getGlobalFrameID(), transform_tolerance_))
  {
    throw nav2_core::ControllerTFError("Unable to transform goal pose into costmap frame");
  }
  return transformed_goal;
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

  return distance <= inversion_xy_tolerance_ && fabs(angle_distance) <= inversion_yaw_tolerance;
}

float PathHandler::getPlanLengthUpToInversion() const
{
  return global_plan_length_up_to_inversion_;
}

float PathHandler::getPlanLength() const
{
  return global_plan_length_;
}

}  // namespace mppi
