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

#include "nav2_util/path_utils.hpp"

#include <limits>
#include <cmath>

#include "nav2_util/geometry_utils.hpp"

namespace nav2_util
{
PathSearchResult distanceFromPath(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  PathSearchResult result;
  result.closest_segment_index = 0;
  result.distance = std::numeric_limits<double>::infinity();

  if (path.poses.empty()) {
    result.distance = std::numeric_limits<double>::infinity();
    return result;
  }

  if (path.poses.size() == 1) {
    result.distance = nav2_util::geometry_utils::distanceToPoint(robot_pose, path.poses.front());
    result.closest_segment_index = 0;
    return result;
  }

  double min_distance = std::numeric_limits<double>::max();
  size_t closest_segment = 0;

  for (size_t i = 0; i < path.poses.size() - 1; ++i) {
    const double current_distance = nav2_util::geometry_utils::distanceToSegment(
      robot_pose,
      path.poses[i],
      path.poses[i + 1]);

    if (current_distance < min_distance) {
      min_distance = current_distance;
      closest_segment = i;  // 🔥 Track closest segment in global search too
    }
  }

  result.distance = min_distance;
  result.closest_segment_index = closest_segment;
  return result;
}

PathSearchResult distanceFromPath(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const size_t start_index,
  const double search_window_length)
{
  PathSearchResult result;
  result.closest_segment_index = start_index;
  result.distance = std::numeric_limits<double>::infinity();

  if (path.poses.empty()) {
    result.distance = std::numeric_limits<double>::infinity();
    return result;
  }

  if (path.poses.size() == 1) {
    result.distance = nav2_util::geometry_utils::distanceToPoint(robot_pose, path.poses.front());
    result.closest_segment_index = 0;
    return result;
  }

  if (start_index >= path.poses.size()) {
    result.distance = std::numeric_limits<double>::infinity();
    return result;
  }

  if (start_index == path.poses.size() - 1) {
    result.distance = nav2_util::geometry_utils::distanceToPoint(robot_pose, path.poses.back());
    result.closest_segment_index = start_index;
    return result;
  }

  if (search_window_length <= 0.0) {
    result.distance = nav2_util::geometry_utils::distanceToPoint(robot_pose,
        path.poses[start_index]);
    result.closest_segment_index = start_index;
    return result;
  }

  double min_distance = std::numeric_limits<double>::max();
  size_t closest_segment = start_index;
  size_t segments_checked = 0;
  const size_t max_segments = static_cast<size_t>(search_window_length);

  for (size_t i = start_index; i < path.poses.size() - 1; ++i) {
    const double current_segment_dist = nav2_util::geometry_utils::distanceToSegment(
      robot_pose,
      path.poses[i],
      path.poses[i + 1]);

    if (current_segment_dist < min_distance) {
      min_distance = current_segment_dist;
      closest_segment = i;
    }

    segments_checked++;
    if (segments_checked >= max_segments) {
      break;
    }
  }

  result.closest_segment_index = closest_segment;
  result.distance = min_distance;
  return result;
}

}  // namespace nav2_util
