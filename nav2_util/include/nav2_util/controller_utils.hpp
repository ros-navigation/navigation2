// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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


#ifndef NAV2_UTIL__CONTROLLER_UTILS_HPP_
#define NAV2_UTIL__CONTROLLER_UTILS_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_util
{
/**
* @brief Find the intersection a circle and a line segment.
* This assumes the circle is centered at the origin.
* If no intersection is found, a floating point error will occur.
* @param p1 first endpoint of line segment
* @param p2 second endpoint of line segment
* @param r radius of circle
* @return point of intersection
*/
geometry_msgs::msg::Point circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  double r);

/**
* @brief Get lookahead point
* @param lookahead_dist Optimal lookahead distance
* @param path Current global path
* @param interpolate_after_goal If true, interpolate the lookahead point after the goal based
* on the orientation given by the position of the last two pose of the path
* @return Lookahead point
*/
geometry_msgs::msg::PoseStamped getLookAheadPoint(
  double &, const nav_msgs::msg::Path &,
  const bool interpolate_after_goal = false);

/**
 * @brief Find the iterator of the first pose at which there is an inversion on the path,
 * @param path to check for inversion
 * @return the first point after the inversion found in the path
 */
inline unsigned int findFirstPathInversion(nav_msgs::msg::Path & path)
{
  // At least 3 poses for a possible inversion
  if (path.poses.size() < 3) {
    return path.poses.size();
  }

  // Iterating through the path to determine the position of the path inversion
  for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    float oa_x = path.poses[idx].pose.position.x -
      path.poses[idx - 1].pose.position.x;
    float oa_y = path.poses[idx].pose.position.y -
      path.poses[idx - 1].pose.position.y;
    float ab_x = path.poses[idx + 1].pose.position.x -
      path.poses[idx].pose.position.x;
    float ab_y = path.poses[idx + 1].pose.position.y -
      path.poses[idx].pose.position.y;

    // Checking for the existence of cusp, in the path, using the dot product.
    float dot_product = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_product < 0.0f) {
      return idx + 1;
    }
  }

  return path.poses.size();
}

/**
 * @brief Find and remove poses after the first inversion in the path
 * @param path to check for inversion
 * @return The location of the inversion, return 0 if none exist
 */
inline unsigned int removePosesAfterFirstInversion(nav_msgs::msg::Path & path)
{
  nav_msgs::msg::Path cropped_path = path;
  const unsigned int first_after_inversion = findFirstPathInversion(cropped_path);
  if (first_after_inversion == path.poses.size()) {
    return 0u;
  }

  cropped_path.poses.erase(
    cropped_path.poses.begin() + first_after_inversion, cropped_path.poses.end());
  path = cropped_path;
  return first_after_inversion;
}

}  // namespace nav2_util

#endif  // NAV2_UTIL__CONTROLLER_UTILS_HPP_
