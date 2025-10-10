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

#include <memory>
#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_controller/plugins/simple_path_handler.hpp"
#include "nav2_util/controller_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_core/controller_exceptions.hpp"


using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{
using nav2_util::geometry_utils::euclidean_distance;

void SimplePathHandler::initialize(
  const nav2::LifecycleNode::WeakPtr & parent,
  const rclcpp::Logger & logger,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  std::shared_ptr<tf2_ros::Buffer> tf)
{
  logger_ = logger;
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  interpolate_curvature_after_goal_ = node->declare_or_get_parameter(plugin_name +
      ".interpolate_curvature_after_goal", false);
  max_robot_pose_search_dist_ = node->declare_or_get_parameter(plugin_name +
      ".max_robot_pose_search_dist", getCostmapMaxExtent());
  enforce_path_inversion_ = node->declare_or_get_parameter(plugin_name + ".enforce_path_inversion",
      false);
  inversion_xy_tolerance_ = node->declare_or_get_parameter(plugin_name + ".inversion_xy_tolerance",
      0.2);
  inversion_yaw_tolerance_ = node->declare_or_get_parameter(plugin_name +
      ".inversion_yaw_tolerance", 0.4);
  node->get_parameter("transform_tolerance", transform_tolerance_);
  if (max_robot_pose_search_dist_ < 0.0) {
    RCLCPP_WARN(
      logger_, "Max robot search distance is negative, setting to max to search"
      " every point on path for the closest value.");
    max_robot_pose_search_dist_ = std::numeric_limits<double>::max();
  }
  if(enforce_path_inversion_) {
    inversion_locale_ = 0u;
  }

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SimplePathHandler::dynamicParametersCallback, this, _1));
}

double SimplePathHandler::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_ros_->getCostmap()->getSizeInCellsX(),
    costmap_ros_->getCostmap()->getSizeInCellsY());
  return max_costmap_dim_meters / 2.0;
}

void SimplePathHandler::prunePlan(nav_msgs::msg::Path & plan, const PathIterator end)
{
  plan.poses.erase(plan.poses.begin(), end);
}

bool SimplePathHandler::isWithinInversionTolerances(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Keep full path if we are within tolerance of the inversion pose
  const auto last_pose = global_plan_up_to_inversion_.poses.back();
  float distance = hypotf(
    robot_pose.pose.position.x - last_pose.pose.position.x,
    robot_pose.pose.position.y - last_pose.pose.position.y);

  float angle_distance = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation),
    tf2::getYaw(last_pose.pose.orientation));

  return distance <= inversion_xy_tolerance_ &&
         fabs(angle_distance) <= inversion_yaw_tolerance_;
}

void SimplePathHandler::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  global_plan_up_to_inversion_ = global_plan_;
  if(enforce_path_inversion_) {
    inversion_locale_ = nav2_util::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
  }
}

geometry_msgs::msg::PoseStamped SimplePathHandler::transformToGlobalPlanFrame(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_up_to_inversion_.poses.empty()) {
    throw nav2_core::InvalidPath("Received plan with zero length");
  }

  if (interpolate_curvature_after_goal_ && global_plan_up_to_inversion_.poses.size() == 1) {
    throw nav2_core::InvalidPath("Received plan with length of one");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::transformPoseInTargetFrame(pose, robot_pose, *tf_,
      global_plan_up_to_inversion_.header.frame_id,
      transform_tolerance_))
  {
    throw nav2_core::ControllerTFError("Unable to transform robot pose into global plan's frame");
  }

  return robot_pose;
}

std::pair<nav_msgs::msg::Path, PathIterator>
SimplePathHandler::getGlobalPlanConsideringBoundsInCostmapFrame(
  const geometry_msgs::msg::PoseStamped & global_pose)
{
  // Limit the search for the closest pose up to max_robot_pose_search_dist on the path
  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_up_to_inversion_.poses.begin(), global_plan_up_to_inversion_.poses.end(),
      max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto closest_point =
    nav2_util::geometry_utils::min_by(
    global_plan_up_to_inversion_.poses.begin(), closest_pose_upper_bound,
    [&global_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(global_pose, ps);
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

  const double max_costmap_extent = getCostmapMaxExtent();
  auto pruned_plan_end = std::find_if(
  closest_point, global_plan_up_to_inversion_.poses.end(),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(global_plan_pose, global_pose) > max_costmap_extent;
  });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_up_to_inversion_.header.frame_id;
      stamped_pose.header.stamp = global_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      if (!nav2_util::transformPoseInTargetFrame(stamped_pose, transformed_pose, *tf_,
        costmap_ros_->getBaseFrameID(), transform_tolerance_))
      {
        throw nav2_core::ControllerTFError("Unable to transform plan pose into local frame");
      }
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    closest_point, pruned_plan_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = global_pose.header.stamp;

  return {transformed_plan, closest_point};
}

nav_msgs::msg::Path SimplePathHandler::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped global_pose = transformToGlobalPlanFrame(pose);
  auto [transformed_plan,
    closest_point] = getGlobalPlanConsideringBoundsInCostmapFrame(global_pose);

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  prunePlan(global_plan_up_to_inversion_, closest_point);

  if (enforce_path_inversion_ && inversion_locale_ != 0u) {
    if (isWithinInversionTolerances(global_pose)) {
      prunePlan(global_plan_, global_plan_.poses.begin() + inversion_locale_);
      global_plan_up_to_inversion_ = global_plan_;
      inversion_locale_ = nav2_util::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
    }
  }

  if (transformed_plan.poses.empty()) {
    throw nav2_core::InvalidPath("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

rcl_interfaces::msg::SetParametersResult
SimplePathHandler::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "max_robot_pose_search_dist") {
        max_robot_pose_search_dist_ = parameter.as_double();
      } else if (param_name == "inversion_xy_tolerance") {
        inversion_xy_tolerance_ = parameter.as_double();
      } else if (param_name == "inversion_yaw_tolerance") {
        inversion_yaw_tolerance_ = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == "enforce_path_inversion") {
        enforce_path_inversion_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::SimplePathHandler, nav2_core::PathHandler)
