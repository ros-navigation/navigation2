// Copyright (c) 2022, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef NAV2_UTIL__SMOOTHER_UTILS_HPP_
#define NAV2_UTIL__SMOOTHER_UTILS_HPP_

#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <utility>

#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "angles/angles.h"
#include "tf2/utils.hpp"

namespace nav2_util
{

/**
 * @class nav2_util::PathSegment
 * @brief A segment of a path in start/end indices
 */
struct PathSegment
{
  unsigned int start;
  unsigned int end;
};

/**
 * @brief Finds the starting and end indices of path segments where
 * the robot is traveling in the same direction (e.g. forward vs reverse)
 * @param path Path in which to look for cusps
 * @param is_holonomic Whether the motion model is holonomic (default is false)
 Only set as true when the input path is known to be generated from a holonomic planner like NavFn.
 * @return Set of index pairs for each segment of the path in a given direction
 */
inline std::vector<PathSegment> findDirectionalPathSegments(
  const nav_msgs::msg::Path & path, bool is_holonomic = false)
{
  std::vector<PathSegment> segments;
  PathSegment curr_segment;
  curr_segment.start = 0;

  // If holonomic, no directional changes and
  // may have abrupt angular changes from naive grid search
  if (is_holonomic) {
    curr_segment.end = path.poses.size() - 1;
    segments.push_back(curr_segment);
    return segments;
  }

  // Iterating through the path to determine the position of the cusp
  for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = path.poses[idx].pose.position.x -
      path.poses[idx - 1].pose.position.x;
    double oa_y = path.poses[idx].pose.position.y -
      path.poses[idx - 1].pose.position.y;
    double ab_x = path.poses[idx + 1].pose.position.x -
      path.poses[idx].pose.position.x;
    double ab_y = path.poses[idx + 1].pose.position.y -
      path.poses[idx].pose.position.y;

    // Checking for the existence of cusp, in the path, using the dot product.
    double dot_product = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_product < 0.0) {
      curr_segment.end = idx;
      segments.push_back(curr_segment);
      curr_segment.start = idx;
    }

    // Checking for the existence of a differential rotation in place.
    double cur_theta = tf2::getYaw(path.poses[idx].pose.orientation);
    double next_theta = tf2::getYaw(path.poses[idx + 1].pose.orientation);
    double dtheta = angles::shortest_angular_distance(cur_theta, next_theta);
    if (fabs(ab_x) < 1e-4 && fabs(ab_y) < 1e-4 && fabs(dtheta) > 1e-4) {
      curr_segment.end = idx;
      segments.push_back(curr_segment);
      curr_segment.start = idx;
    }
  }

  curr_segment.end = path.poses.size() - 1;
  segments.push_back(curr_segment);
  return segments;
}

/**
 * @brief For a given path, update the path point orientations based on smoothing
 * @param path Path to approximate the path orientation in
 * @param reversing_segment Return if this is a reversing segment
 * @param is_holonomic Whether the motion model is holonomic (default is false)
 Only set as true when the input path is known to be generated from a holonomic planner like NavFn.
 */
inline void updateApproximatePathOrientations(
  nav_msgs::msg::Path & path,
  bool & reversing_segment,
  bool is_holonomic = false)
{
  double dx, dy, theta, pt_yaw;
  reversing_segment = false;

  // Find if this path segment is in reverse
  dx = path.poses[2].pose.position.x - path.poses[1].pose.position.x;
  dy = path.poses[2].pose.position.y - path.poses[1].pose.position.y;
  theta = atan2(dy, dx);
  pt_yaw = tf2::getYaw(path.poses[1].pose.orientation);
  if (!is_holonomic && fabs(angles::shortest_angular_distance(pt_yaw, theta)) > M_PI_2) {
    reversing_segment = true;
  }

  // Find the angle relative the path position vectors
  for (unsigned int i = 0; i != path.poses.size() - 1; i++) {
    dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    theta = atan2(dy, dx);

    // If points are overlapping, pass
    if (fabs(dx) < 1e-4 && fabs(dy) < 1e-4) {
      continue;
    }

    // Flip the angle if this path segment is in reverse
    if (reversing_segment) {
      theta += M_PI;  // orientationAroundZAxis will normalize
    }

    path.poses[i].pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
  }
}

}  // namespace nav2_util

#endif  // NAV2_UTIL__SMOOTHER_UTILS_HPP_
