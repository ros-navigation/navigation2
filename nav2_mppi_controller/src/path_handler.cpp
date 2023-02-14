// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

namespace mppi
{

void PathHandler::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
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
}

PathRange PathHandler::getGlobalPlanConsideringBounds(
  const geometry_msgs::msg::PoseStamped & global_pose)
{
  using nav2_util::geometry_utils::euclidean_distance;
  auto begin = global_plan_.poses.begin();
  auto end = global_plan_.poses.end();

  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // Find closest point to the robot
  auto closest_point = nav2_util::geometry_utils::min_by(
    begin, closest_pose_upper_bound,
    [&global_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(global_pose, ps);
    });

  // Find the furthest relevent point on the path to consider within costmap
  // bounds
  const auto * costmap = costmap_->getCostmap();
  unsigned int mx, my;
  auto last_point =
    std::find_if(
    closest_point, end, [&](const geometry_msgs::msg::PoseStamped & global_plan_pose) {
      auto distance = euclidean_distance(global_pose, global_plan_pose);
      return distance >= prune_distance_ || !costmap->worldToMap(
        global_plan_pose.pose.position.x, global_plan_pose.pose.position.y, mx, my);
    });

  return {closest_point, last_point};
}

geometry_msgs::msg::PoseStamped PathHandler::transformToGlobalPlanFrame(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("Received plan with zero length");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::ControllerTFError(
            "Unable to transform robot pose into global plan's frame");
  }

  return robot_pose;
}

nav_msgs::msg::Path PathHandler::transformPath(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Find relevent bounds of path to use
  geometry_msgs::msg::PoseStamped global_pose =
    transformToGlobalPlanFrame(robot_pose);
  auto [lower_bound, upper_bound] = getGlobalPlanConsideringBounds(global_pose);

  // Transform these bounds into the local costmap frame and prune older points
  const auto & stamp = global_pose.header.stamp;
  nav_msgs::msg::Path transformed_plan =
    transformPlanPosesToCostmapFrame(lower_bound, upper_bound, stamp);

  pruneGlobalPlan(lower_bound);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::InvalidPath("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool PathHandler::transformPose(
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_buffer_->transform(
      in_pose, out_pose, frame,
      tf2::durationFromSec(transform_tolerance_));
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double PathHandler::getMaxCostmapDist()
{
  const auto & costmap = costmap_->getCostmap();
  return std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
         costmap->getResolution() / 2.0;
}

nav_msgs::msg::Path PathHandler::transformPlanPosesToCostmapFrame(
  PathIterator begin, PathIterator end, const builtin_interfaces::msg::Time & stamp)
{
  std::string frame = costmap_->getGlobalFrameID();
  auto transformToFrame = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped from_pose;
      geometry_msgs::msg::PoseStamped to_pose;

      from_pose.header.frame_id = global_plan_.header.frame_id;
      from_pose.header.stamp = stamp;
      from_pose.pose = global_plan_pose.pose;

      transformPose(frame, from_pose, to_pose);
      return to_pose;
    };

  nav_msgs::msg::Path plan;
  plan.header.frame_id = frame;
  plan.header.stamp = stamp;

  std::transform(begin, end, std::back_inserter(plan.poses), transformToFrame);

  return plan;
}

void PathHandler::setPath(const nav_msgs::msg::Path & plan)
{
  global_plan_ = plan;
}

nav_msgs::msg::Path & PathHandler::getPath() {return global_plan_;}

void PathHandler::pruneGlobalPlan(const PathIterator end)
{
  global_plan_.poses.erase(global_plan_.poses.begin(), end);
}

}  // namespace mppi
