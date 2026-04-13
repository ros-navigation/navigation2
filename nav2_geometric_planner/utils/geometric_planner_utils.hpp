// Copyright (c) 2025 Nav2 Contributors
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

#ifndef NAV2_GEOMETRIC_PLANNERS__GEOMETRIC_PLANNER_UTILS_HPP_
#define NAV2_GEOMETRIC_PLANNERS__GEOMETRIC_PLANNER_UTILS_HPP_

#include <cmath>
#include <string>

#include "tf2/utils.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_core/planner_exceptions.hpp"

namespace nav2_geometric_planners
{
namespace utils
{

/**
 * @brief Extract yaw angle from a quaternion message.
 * @param q Quaternion to extract yaw from.
 * @return Yaw angle in radians.
 */
inline double extractYaw(const geometry_msgs::msg::Quaternion & q)
{
  return tf2::getYaw(q);
}

/**
 * @brief Build a PoseStamped from position and heading.
 * @param x X position.
 * @param y Y position.
 * @param yaw Heading in radians.
 * @param header Header to stamp the pose with.
 * @return Constructed PoseStamped.
 */
inline geometry_msgs::msg::PoseStamped makePose(
  double x, double y, double yaw,
  const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;
  pose.pose.position.x = x;
  pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  return pose;
}

/**
 * @brief Compute Euclidean distance between two poses.
 * @param a First pose.
 * @param b Second pose.
 * @return Distance in metres.
 */
inline double poseDistance(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  const double dx = b.pose.position.x - a.pose.position.x;
  const double dy = b.pose.position.y - a.pose.position.y;
  return std::hypot(dx, dy);
}

/**
 * @brief Test whether a world coordinate is collision-free in the costmap.
 * @param x World x coordinate.
 * @param y World y coordinate.
 * @param costmap Pointer to the 2-D costmap.
 * @return True if the cell is below LETHAL_OBSTACLE cost and inside the map.
 */
inline bool isStateValid(double x, double y, nav2_costmap_2d::Costmap2D * costmap)
{
  unsigned int mx{}, my{};
  if (!costmap->worldToMap(x, y, mx, my)) {
    return false;
  }
  return costmap->getCost(mx, my) < nav2_costmap_2d::LETHAL_OBSTACLE;
}

/**
 * @brief Validate that start and goal poses are navigable.
 *
 * Throws nav2_core::StartOutsideMapBounds, nav2_core::GoalOutsideMapBounds,
 * nav2_core::StartOccupied, or nav2_core::GoalOccupied as appropriate.
 *
 * @param start Start pose.
 * @param goal  Goal pose.
 * @param costmap Pointer to the 2-D costmap.
 */
inline void validateStartGoal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav2_costmap_2d::Costmap2D * costmap)
{
  if (!isStateValid(start.pose.position.x, start.pose.position.y, costmap)) {
    unsigned int mx{}, my{};
    if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)) {
      throw nav2_core::StartOutsideMapBounds("Start pose is outside map bounds");
    }
    throw nav2_core::StartOccupied("Start pose is occupied");
  }
  if (!isStateValid(goal.pose.position.x, goal.pose.position.y, costmap)) {
    unsigned int mx{}, my{};
    if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
      throw nav2_core::GoalOutsideMapBounds("Goal pose is outside map bounds");
    }
    throw nav2_core::GoalOccupied("Goal pose is occupied");
  }
}

/**
 * @brief Scan a path for collisions and throw if any are found.
 *
 * Throws nav2_core::NoValidPathCouldBeFound if any pose in the path
 * is in collision or outside the map.
 *
 * @param path    The candidate path to check.
 * @param costmap Pointer to the 2-D costmap.
 */
inline void checkPathCollisions(
  const nav_msgs::msg::Path & path,
  nav2_costmap_2d::Costmap2D * costmap)
{
  for (const auto & pose : path.poses) {
    if (!isStateValid(pose.pose.position.x, pose.pose.position.y, costmap)) {
      throw nav2_core::NoValidPathCouldBeFound("Path passes through a collision");
    }
  }
}

}  // namespace utils
}  // namespace nav2_geometric_planners

#endif  // NAV2_GEOMETRIC_PLANNERS__GEOMETRIC_PLANNER_UTILS_HPP_
