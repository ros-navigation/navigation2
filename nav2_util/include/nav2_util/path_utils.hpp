/******************************************************************************  
 *  Copyright (c) 2025, Berkan Tali
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *****************************************************************************/

#ifndef NAV2_UTIL__PATH_UTILS_HPP_
#define NAV2_UTIL__PATH_UTILS_HPP_

#include <cstddef>   // size_t
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_util
{

/**
 * @brief  Lateral (XY-plane) distance between a robot pose and a path.
 *
 * Implements the “iterative local minimum” strategy requested in
 * <https://github.com/ros-navigation/navigation2/issues/5037>.
 *   * First call, @p closest_idx is nullptr ➜ full global scan.  
 *   * Subsequent calls pass the prior index ➜ local scan forward only,
 *     giving stable behaviour on self-intersecting loops.
 *
 * Closed paths (first ≈ last point) are handled automatically.
 *
 * @param pose         Robot pose (must be in same frame as @p path).
 * @param path         Path to evaluate.
 * @param closest_idx  [in/out] Pointer to the starting segment index; may be
 *                     nullptr.  On return, updated with the segment that
 *                     achieved the minimum.
 *
 * @return  Shortest distance in metres, or +∞ if @p path is empty.
 *
 * @throws std::invalid_argument if @p pose.header.frame_id differs from
 *                               @p path.header.frame_id.
 */
double distanceFromPath(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path &             path,
  size_t *                                closest_idx = nullptr);

}  // namespace nav2_util

#endif  // NAV2_UTIL__PATH_UTILS_HPP_
