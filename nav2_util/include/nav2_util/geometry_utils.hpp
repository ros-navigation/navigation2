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

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/path.hpp"

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
 * @brief Get the L2 distance between 2 geometry_msgs::Pose2D
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::Pose2D & pos1,
  const geometry_msgs::msg::Pose2D & pos2)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;

  return std::hypot(dx, dy);
}

/**
 * @brief Get the squared euclidean distance between 2 geometry_msgs::Poses
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true 3D distance is desired (default false)
 * @return double squared euclidean distance
 */
inline double squared_euclidean_distance(
  const geometry_msgs::msg::Pose & pos1,
  const geometry_msgs::msg::Pose & pos2,
  const bool is_3d = false)
{
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;

  if (is_3d) {
    double dz = pos1.position.z - pos2.position.z;
    return dx * dx + dy * dy + dz * dz;
  }

  return dx * dx + dy * dy;
}

/**
 * @brief Get the squared L2 distance between 2 geometry_msgs::PoseStamped
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true squared L2 distance is desired (default false)
 * @return double squared L2 distance
 */
inline double squared_euclidean_distance(
  const geometry_msgs::msg::PoseStamped & pos1,
  const geometry_msgs::msg::PoseStamped & pos2,
  const bool is_3d = false)
{
  return squared_euclidean_distance(pos1.pose, pos2.pose, is_3d);
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
    if (comp < lowest) {
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

std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped>
find_closest_path_segment(const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & pose)
{
  std::vector<double> distances(path.poses.size());
  std::transform(
    path.poses.begin(),
    path.poses.end(),
    distances.begin(),
    [&pose](const auto & path_pose) {
      return squared_euclidean_distance(path_pose, pose);
    }
  );
  auto closest_distance_it = std::min_element(
    distances.begin(),
    distances.end()
  );

  auto next_closest_distance_it = distances.end();

  if (std::next(closest_distance_it) == distances.end()) {
    next_closest_distance_it = std::prev(closest_distance_it);
  } else if (closest_distance_it == distances.begin()) {
    next_closest_distance_it = std::next(closest_distance_it);
  } else if (*std::prev(closest_distance_it) < *std::next(closest_distance_it)) {
    next_closest_distance_it = std::prev(closest_distance_it);
  } else {
    next_closest_distance_it = std::next(closest_distance_it);
  }

  size_t closest_pose_index = std::distance(distances.begin(), closest_distance_it);
  size_t next_closest_pose_index = std::distance(distances.begin(), next_closest_distance_it);

  return std::make_pair(path.poses.at(closest_pose_index), path.poses.at(next_closest_pose_index));
}

double cross_track_error(const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & pose)
{
  // http://paulbourke.net/geometry/pointlineplane/

  // Even if the target pose is "off the end" of this line segment, the projected point
  // exists as if the segment was extended out.
  const auto & [pose1, pose2] = find_closest_path_segment(path, pose);
  const double x1 = pose1.pose.position.x;
  const double y1 = pose1.pose.position.y;
  const double x2 = pose2.pose.position.x;
  const double y2 = pose2.pose.position.y;
  const double x3 = pose.pose.position.x;
  const double y3 = pose.pose.position.y;

  const double u = ((x3 - x1) * (x2 - x1) + (y3 - y1) * (y2 - y1)) / ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

  const double x = x1 + u * (x2 - x1);
  const double y = y1 + u * (y2 - y1);

  return std::sqrt((x - x3) * (x - x3) + (y - y3) * (y - y3));
}

}  // namespace geometry_utils
}  // namespace nav2_util

#endif  // NAV2_UTIL__GEOMETRY_UTILS_HPP_
