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
#include "tf2/utils.hpp"
#include "angles/angles.h"
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

/**
 * @brief Find the iterator of the first pose at which there is an inversion or in place rotation on the path,
 * @param path to check for inversion or rotation
 * @param enforce_path_inversion Whether to enable check for inversion
 * @param rotation_threshold Minimum rotation angle to consider an in-place rotation (0 to disable rotation check)
 * @return the first point after the inversion or in place rotation found in the path
 */
inline unsigned int findFirstPathConstraint(
  nav_msgs::msg::Path & path,
  bool enforce_path_inversion,
  float rotation_threshold)
{
  // At least 3 poses for a possible inversion
  if (path.poses.size() < 3) {
    return path.poses.size();
  }

  const bool check_rotation = fabs(rotation_threshold) < 1e-6f ? false : true;
  unsigned int rotation_idx = path.poses.size();
  unsigned int inversion_idx = path.poses.size();
  float prev_dx = 0.0f;
  float prev_dy = 0.0f;
  bool prev_valid = false;

  // Iterating through the path to determine the position of the path inversion or rotation
  for (unsigned int idx = 0; idx < path.poses.size() - 1; ++idx) {
    float dx = path.poses[idx + 1].pose.position.x -
      path.poses[idx].pose.position.x;
    float dy = path.poses[idx + 1].pose.position.y -
      path.poses[idx].pose.position.y;
    float trans = hypot(dx, dy);

    if (rotation_idx <= idx + 1 && inversion_idx <= idx + 1) {
      break;
    }

    // Check inversion
    if (enforce_path_inversion && trans > 1e-4) {
      if (prev_valid) {
        // Checking for the existence of cusp, in the path, using the dot product.
        float dot_product = prev_dx * dx + prev_dy * dy;
        if (dot_product < 0.0f) {
          inversion_idx = std::min(inversion_idx, idx + 1);
        }
      }
      prev_dx = dx;
      prev_dy = dy;
      prev_valid = true;
    }

    // Check in place rotation
    if (check_rotation && trans < 1e-4 && rotation_idx == path.poses.size()) {
      float accumulated_rotation = 0.0f;
      unsigned int end_idx = idx;

      // Continue checking while translation remains small
      // until accumulated rotation is larger than threshold
      while (end_idx < path.poses.size() - 1) {
        float current_yaw = tf2::getYaw(path.poses[end_idx].pose.orientation);
        float next_yaw = tf2::getYaw(path.poses[end_idx + 1].pose.orientation);
        accumulated_rotation += fabs(angles::shortest_angular_distance(current_yaw, next_yaw));
        if (accumulated_rotation > rotation_threshold) {
          rotation_idx = end_idx + 1;
          break;
        }
        if (end_idx + 2 < path.poses.size()) {
          float ndx = path.poses[end_idx + 2].pose.position.x -
            path.poses[end_idx + 1].pose.position.x;
          float ndy = path.poses[end_idx + 2].pose.position.y -
            path.poses[end_idx + 1].pose.position.y;
          // Stop if translation resumes
          if (hypot(ndx, ndy) > 1e-4) {
            break;
          }
        } else {
          // We have reached the end of the path
          break;
        }
        end_idx++;
      }
    }
  }

  return std::min(rotation_idx, inversion_idx);
}

/**
 * @brief Find and remove poses after the first constraint in the path
 * @param path to check for inversion or rotation
 * @param enforce_path_inversion Whether to enable check for inversion
 * @param rotation_threshold Minimum rotation angle to consider an in-place rotation (0 to disable rotation check)
 * @return The location of the inversion or rotation, return 0 if none exist
 */
inline unsigned int removePosesAfterFirstConstraint(
  nav_msgs::msg::Path & path,
  bool enforce_path_inversion,
  float rotation_threshold)
{
  auto start = std::chrono::system_clock::now();
  nav_msgs::msg::Path cropped_path = path;
  const unsigned int first_after_constraint = findFirstPathConstraint(cropped_path,
    enforce_path_inversion, rotation_threshold);
  if (first_after_constraint == path.poses.size()) {
    return 0u;
  }

  cropped_path.poses.erase(
    cropped_path.poses.begin() + first_after_constraint, cropped_path.poses.end());
  path = cropped_path;
  auto end = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  // RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
  std::cout << "Control loop execution time:" << duration << std::endl;
  return first_after_constraint;
}

}  // namespace nav2_util

#endif  // NAV2_UTIL__PATH_UTILS_HPP_
