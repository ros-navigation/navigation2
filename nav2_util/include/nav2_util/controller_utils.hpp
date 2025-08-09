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

}  // namespace nav2_util

#endif  // NAV2_UTIL__CONTROLLER_UTILS_HPP_
