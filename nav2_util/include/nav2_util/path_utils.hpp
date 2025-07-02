// Copyright (c) 2024, Berkan Tali
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

#ifndef NAV2_UTIL__PATH_UTILS_HPP_
#define NAV2_UTIL__PATH_UTILS_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_util
{

/**
 * @brief Compute the shortest lateral (XY) distance from a pose to a path.
 *
 * This function is optimized for repeated calls by allowing an optional starting
 * search index. If the index is provided, the search for the closest segment
 * begins from that point, and the index is updated with the new closest
 * segment's starting index for the next call. This provides stable and efficient
 * behavior, especially for controllers following a path. It also correctly
 * handles closed-loop paths.
 *
 * @param pose The pose from which to measure the distance.
 * @param path The path to measure the distance to.
 * @param closest_idx Optional in/out pointer to the starting index for the search.
 * If nullptr, a full global search is performed.
 * @return The shortest distance in meters from the pose to the path.
 * Returns +infinity for an empty path.
 */
double distanceFromPath(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & path,
  size_t * closest_idx = nullptr);

}  // namespace nav2_util

#endif  // NAV2_UTIL__PATH_UTILS_HPP_
