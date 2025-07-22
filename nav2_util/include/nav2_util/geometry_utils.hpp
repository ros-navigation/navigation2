// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_UTIL__GEOMETRY_UTILS_HPP_
#define NAV2_UTIL__GEOMETRY_UTILS_HPP_

#include <cmath>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/waypoint_status.hpp"

namespace nav2_util
{
namespace geometry_utils
{

/**
 * @brief Get a geometry_msgs Quaternion from a yaw angle
 * @param angle Yaw angle to generate a quaternion from
 * @return geometry_msgs Quaternion
 */
inline geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);  // void returning function
  return tf2::toMsg(q);
}

/**
 * @brief Get the euclidean distance between 2 geometry_msgs::Points
 * @param pos1 First point
 * @param pos1 Second point
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::Point & pos1,
  const geometry_msgs::msg::Point & pos2,
  const bool is_3d = false)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;

  if (is_3d) {
    double dz = pos1.z - pos2.z;
    return std::hypot(dx, dy, dz);
  }

  return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Poses
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double euclidean distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::Pose & pos1,
  const geometry_msgs::msg::Pose & pos2,
  const bool is_3d = false)
{
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;

  if (is_3d) {
    double dz = pos1.position.z - pos2.position.z;
    return std::hypot(dx, dy, dz);
  }

  return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::PoseStamped
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::PoseStamped & pos1,
  const geometry_msgs::msg::PoseStamped & pos2,
  const bool is_3d = false)
{
  return euclidean_distance(pos1.pose, pos2.pose, is_3d);
}

/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
inline Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp <= lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

/**
 * Find first element in iterator that is greater integrated distance than comparevalue
 */
template<typename Iter, typename Getter>
inline Iter first_after_integrated_distance(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  Getter dist = 0.0;
  for (Iter it = begin; it != end - 1; it++) {
    dist += euclidean_distance(*it, *(it + 1));
    if (dist > getCompareVal) {
      return it + 1;
    }
  }
  return end;
}

/**
 * @brief Calculate the length of the provided path, starting at the provided index
 * @param path Path containing the poses that are planned
 * @param start_index Optional argument specifying the starting index for
 * the calculation of path length. Provide this if you want to calculate length of a
 * subset of the path.
 * @return double Path length
 */
inline double calculate_path_length(const nav_msgs::msg::Path & path, size_t start_index = 0)
{
  if (start_index + 1 >= path.poses.size()) {
    return 0.0;
  }
  double path_length = 0.0;
  for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
    path_length += euclidean_distance(path.poses[idx].pose, path.poses[idx + 1].pose);
  }
  return path_length;
}

/**
 * @brief Find the index of the first goal in `PENDING` status matching the
 * given target pose.
 * @param waypoint_statuses List of waypoint statuses to search through.
 * @param goal Target pose to match against waypoint goals.
 * @return Index of the first matching goal in PENDING status, -1 if not found.
 */
inline int find_next_matching_goal_in_waypoint_statuses(
  const std::vector<nav2_msgs::msg::WaypointStatus> & waypoint_statuses,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto itr = std::find_if(waypoint_statuses.begin(), waypoint_statuses.end(),
      [&goal](const nav2_msgs::msg::WaypointStatus & status){
        return status.waypoint_pose == goal &&
               status.waypoint_status == nav2_msgs::msg::WaypointStatus::PENDING;
    });

  if (itr == waypoint_statuses.end()) {
    return -1;
  }

  return itr - waypoint_statuses.begin();
}

/**
 * @brief Checks if point is inside the polygon
 * @param px X-coordinate of the given point to check
 * @param py Y-coordinate of the given point to check
 * @param polygon Polygon to check if the point is inside
 * @return True if given point is inside polygon, otherwise false
 */
template<class PointT>
inline bool isPointInsidePolygon(
  const double px, const double py, const std::vector<PointT> & polygon)
{
  // Adaptation of Shimrat, Moshe. "Algorithm 112: position of point relative to polygon."
  // Communications of the ACM 5.8 (1962): 434.
  // Implementation of ray crossings algorithm for point in polygon task solving.
  // Y coordinate is fixed. Moving the ray on X+ axis starting from given point.
  // Odd number of intersections with polygon boundaries means the point is inside polygon.
  const int points_num = polygon.size();
  int i, j;  // Polygon vertex iterators
  bool res = false;  // Final result, initialized with already inverted value

  // Starting from the edge where the last point of polygon is connected to the first
  i = points_num - 1;
  for (j = 0; j < points_num; j++) {
    // Checking the edge only if given point is between edge boundaries by Y coordinates.
    // One of the condition should contain equality in order to exclude the edges
    // parallel to X+ ray.
    if ((py <= polygon[i].y) == (py > polygon[j].y)) {
      // Calculating the intersection coordinate of X+ ray
      const double x_inter = polygon[i].x +
        (py - polygon[i].y) *
        (polygon[j].x - polygon[i].x) /
        (polygon[j].y - polygon[i].y);
      // If intersection with checked edge is greater than point x coordinate,
      // inverting the result
      if (x_inter > px) {
        res = !res;
      }
    }
    i = j;
  }
  return res;
}

/**
 * @brief Find the distance to a point
 * @param global_pose Robot's current or planned position
 * @param target target point
 * @return int
 */
inline double distanceToPoint(
  const geometry_msgs::msg::PoseStamped & point1,
  const geometry_msgs::msg::PoseStamped & point2)
{
  const double dx = point1.pose.position.x - point2.pose.position.x;
  const double dy = point1.pose.position.y - point2.pose.position.y;
  return std::hypot(dx, dy);
}

/**
 * @brief Find the shortest distance to a vector
 * @param global_pose Robot's current or planned position
 * @param start Starting point of target vector
 * @param finish End point of target vector
 * @return int
 */
inline double distanceToSegment(
  const geometry_msgs::msg::PoseStamped & point,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end)
{
  const auto & p = point.pose.position;
  const auto & a = start.pose.position;
  const auto & b = end.pose.position;

  const double dx_seg = b.x - a.x;
  const double dy_seg = b.y - a.y;

  const double seg_len_sq = (dx_seg * dx_seg) + (dy_seg * dy_seg);

  if (seg_len_sq <= 1e-9) {
    return distanceToPoint(point, start);
  }

  const double dot = ((p.x - a.x) * dx_seg) + ((p.y - a.y) * dy_seg);
  const double t = std::clamp(dot / seg_len_sq, 0.0, 1.0);

  const double proj_x = a.x + t * dx_seg;
  const double proj_y = a.y + t * dy_seg;

  const double dx_proj = p.x - proj_x;
  const double dy_proj = p.y - proj_y;
  return std::hypot(dx_proj, dy_proj);
}

}  // namespace geometry_utils
}  // namespace nav2_util

#endif  // NAV2_UTIL__GEOMETRY_UTILS_HPP_
