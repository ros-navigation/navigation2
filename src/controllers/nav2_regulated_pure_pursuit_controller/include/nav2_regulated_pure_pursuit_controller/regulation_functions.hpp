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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATION_FUNCTIONS_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATION_FUNCTIONS_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_regulated_pure_pursuit_controller/parameter_handler.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

namespace heuristics
{

/**
 * @brief apply curvature constraint regulation on the linear velocity
 * @param raw_linear_velocity Raw linear velocity desired
 * @param curvature Curvature of the current command to follow the path
 * @param min_radius Minimum path radius to apply the heuristic
 * @return Velocity after applying the curvature constraint
 */
inline double curvatureConstraint(
  const double raw_linear_vel, const double curvature, const double min_radius)
{
  const double radius = fabs(1.0 / curvature);
  if (radius < min_radius) {
    return raw_linear_vel * (1.0 - (fabs(radius - min_radius) / min_radius));
  } else {
    return raw_linear_vel;
  }
}

/**
 * @brief apply cost constraint regulation on the linear velocity
 * @param raw_linear_velocity Raw linear velocity desired
 * @param pose_cost Cost at the robot pose
 * @param costmap_ros Costmap object to query
 * @param params Parameters
 * @return Velocity after applying the curvature constraint
 */
inline double costConstraint(
  const double raw_linear_vel,
  const double pose_cost,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  Parameters * params)
{
  using namespace nav2_costmap_2d;  // NOLINT

  if (pose_cost != static_cast<double>(NO_INFORMATION) &&
    pose_cost != static_cast<double>(FREE_SPACE))
  {
    const double & inscribed_radius = costmap_ros->getLayeredCostmap()->getInscribedRadius();

    const double min_distance_to_obstacle =
      (params->inflation_cost_scaling_factor * inscribed_radius - log(pose_cost) + log(253.0f)) /
      params->inflation_cost_scaling_factor;

    if (min_distance_to_obstacle < params->cost_scaling_dist) {
      return raw_linear_vel *
             (params->cost_scaling_gain * min_distance_to_obstacle / params->cost_scaling_dist);
    }
  }

  return raw_linear_vel;
}

/**
 * @brief Compute the scale factor to apply for linear velocity regulation on approach to goal
 * @param transformed_path Path to use to calculate distances to goal
 * @param approach_velocity_scaling_dist Minimum distance away to which to apply the heuristic
 * @return A scale from 0.0-1.0 of the distance to goal scaled by minimum distance
 */
inline double approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path,
  const double approach_velocity_scaling_dist)
{
  using namespace nav2_util::geometry_utils;  // NOLINT

  // Waiting to apply the threshold based on integrated distance ensures we don't
  // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
  const double remaining_distance = calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist) {
    auto & last = transformed_path.poses.back();
    // Here we will use a regular euclidean distance from the robot frame (origin)
    // to get smooth scaling, regardless of path density.
    return std::hypot(last.pose.position.x, last.pose.position.y) / approach_velocity_scaling_dist;
  } else {
    return 1.0;
  }
}

/**
 * @brief Velocity on approach to goal heuristic regulation term
 * @param constrained_linear_vel Linear velocity already constrained by heuristics
 * @param path The path plan in the robot base frame coordinates
 * @param min_approach_velocity Minimum velocity to use on approach to goal
 * @param approach_velocity_scaling_dist Distance away from goal to start applying this heuristic
 * @return Velocity after regulation via approach to goal slow-down
 */
inline double approachVelocityConstraint(
  const double constrained_linear_vel,
  const nav_msgs::msg::Path & path,
  const double min_approach_velocity,
  const double approach_velocity_scaling_dist)
{
  double velocity_scaling = approachVelocityScalingFactor(path, approach_velocity_scaling_dist);
  double approach_vel = constrained_linear_vel * velocity_scaling;

  if (approach_vel < min_approach_velocity) {
    approach_vel = min_approach_velocity;
  }

  return std::min(constrained_linear_vel, approach_vel);
}

}  // namespace heuristics

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATION_FUNCTIONS_HPP_
