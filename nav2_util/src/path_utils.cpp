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
#include <stdexcept>

#include "nav2_util/geometry_utils.hpp"
namespace nav2_util
{

PathSearchResult distance_from_path(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Pose & robot_pose,
  const size_t start_index,
  const double search_window_length)
{
  PathSearchResult result;
  result.closest_segment_index = start_index;
  result.distance = std::numeric_limits<double>::max();

  if (path.poses.empty()) {
    return result;
  }

  if (path.poses.size() == 1) {
    result.distance = nav2_util::geometry_utils::euclidean_distance(
      robot_pose, path.poses.front().pose);
    result.closest_segment_index = 0;
    return result;
  }

  if (start_index >= path.poses.size()) {
    throw std::invalid_argument(
      "Invalid operation: requested start index (" + std::to_string(start_index) +
      ") is greater than or equal to path size (" + std::to_string(path.poses.size()) +
      "). Application is not properly managing state.");
  }

  double distance_traversed = 0.0;
  for (size_t i = start_index; i < path.poses.size() - 1; ++i) {
    if (distance_traversed > search_window_length) {
      break;
    }

    const double current_distance = geometry_utils::distance_to_segment(
      robot_pose.position,
      path.poses[i].pose,
      path.poses[i + 1].pose);

    if (current_distance < result.distance) {
      result.distance = current_distance;
      result.closest_segment_index = i;
    }

    distance_traversed += geometry_utils::euclidean_distance(
      path.poses[i],
      path.poses[i + 1]);
  }

  const auto & segment_start = path.poses[result.closest_segment_index];
  const auto & segment_end = path.poses[result.closest_segment_index + 1];

  const double cross_product = geometry_utils::cross_product_2d(
    robot_pose.position, segment_start.pose, segment_end.pose);

  result.distance *= (cross_product >= 0.0 ? 1.0 : -1.0);

  return result;
}

}  // namespace nav2_util
