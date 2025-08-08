// Copyright (c) 2025 Berkan Tali
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

#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/waypoint_status.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_util
{

/**
 * @brief Result of searching for the closest segment on a path.
 */
struct PathSearchResult
{
  double distance;
  size_t closest_segment_index;
};

PathSearchResult distance_from_path(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const size_t start_index = 0,
  const double search_window_length = std::numeric_limits<double>::max());

}  // namespace nav2_util

#endif  // NAV2_UTIL__PATH_UTILS_HPP_
