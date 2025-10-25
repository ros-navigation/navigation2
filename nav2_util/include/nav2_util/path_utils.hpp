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
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
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
/**
 * @brief Finds the minimum distance from the robot's pose to the closest segment of a path.
 *
 * This function searches for the closest segment on the given path to the robot's pose.
 * By default, it finds the globally nearest path point. Optionally, you can specify
 * the index to start searching from (useful for path tracking tasks) and a maximum
 * search window length (in meters) to limit the search range.
 *
 * @param path The path to search (sequence of poses).
 * @param robot_pose The robot's current pose.
 * @param start_index The index in the path to start searching from (default: 0).
 * @param search_window_length The maximum length (in meters) to search along the path (default: unlimited).
 * @return PathSearchResult Struct containing the minimum distance and the index of the closest segment.
 */
PathSearchResult distance_from_path(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Pose & robot_pose,
  const size_t start_index = 0,
  const double search_window_length = std::numeric_limits<double>::max());

/**
  * @brief get an arbitrary path in a target frame
  * @param input_path Path to transform
  * @param transformed_path Output transformation
  * @param tf_buffer TF buffer to use for the transformation
  * @param target_frame Frame to transform into
  * @param transform_timeout TF Timeout to use for transformation
  * @return bool Whether it could be transformed successfully
  */
bool transformPathInTargetFrame(
  const nav_msgs::msg::Path & input_path,
  nav_msgs::msg::Path & transformed_path,
  tf2_ros::Buffer & tf_buffer, const std::string target_frame,
  const double transform_timeout = 0.1);

}  // namespace nav2_util

#endif  // NAV2_UTIL__PATH_UTILS_HPP_
