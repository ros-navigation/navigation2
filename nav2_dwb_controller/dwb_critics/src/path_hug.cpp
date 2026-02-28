// Copyright (c) 2025, Berkan Tali
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

#include "dwb_critics/path_hug.hpp"

#include <vector>
#include <string>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/path_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace dwb_critics
{

void PathHugCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".search_window", rclcpp::ParameterValue(3.0));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".search_window", search_window_);

  if (search_window_ <= 0.0) {
    throw std::runtime_error{"search_window must be positive"};
  }
}

bool PathHugCritic::prepare(
  const geometry_msgs::msg::Pose & /*pose*/, const nav_2d_msgs::msg::Twist2D & /*vel*/,
  const geometry_msgs::msg::Pose & /*goal*/,
  const nav_msgs::msg::Path & global_plan)
{
  global_path_ = global_plan;
  return true;
}

PathHugCritic::SegmentSearchResult PathHugCritic::findClosestSegmentWithLookback(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Pose & pose,
  size_t hint_index,
  double search_window) const
{
  if (path.poses.empty()) {
    return {0, 0.0};
  }

  const size_t path_size = path.poses.size();

  if (path_size == 1) {
    double dist = nav2_util::geometry_utils::euclidean_distance(
      pose.position, path.poses[0].pose.position);
    return {0, dist};
  }

  if (hint_index >= path_size) {
    hint_index = path_size - 1;
  }

  double min_distance = std::numeric_limits<double>::max();
  size_t closest_index = hint_index;
  const double half_window = search_window / 2.0;

  // Search backward from hint_index
  double distance_traversed = 0.0;
  if (hint_index > 0) {
    for (size_t i = hint_index; i > 0; --i) {
      if (distance_traversed > half_window) {
        break;
      }

      const double dist = nav2_util::geometry_utils::distance_to_path_segment(
        pose.position,
        path.poses[i - 1].pose,
        path.poses[i].pose);

      if (dist < min_distance) {
        min_distance = dist;
        closest_index = i - 1;
      }

      distance_traversed += nav2_util::geometry_utils::euclidean_distance(
        path.poses[i - 1].pose.position,
        path.poses[i].pose.position);
    }
  }

  // Search forward from hint_index
  distance_traversed = 0.0;
  for (size_t i = hint_index; i < path_size - 1; ++i) {
    if (distance_traversed > half_window) {
      break;
    }

    const double dist = nav2_util::geometry_utils::distance_to_path_segment(
      pose.position,
      path.poses[i].pose,
      path.poses[i + 1].pose);

    if (dist < min_distance) {
      min_distance = dist;
      closest_index = i;
    }

    distance_traversed += nav2_util::geometry_utils::euclidean_distance(
      path.poses[i].pose.position,
      path.poses[i + 1].pose.position);
  }

  return {closest_index, min_distance};
}

double PathHugCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (traj.poses.empty() || global_path_.poses.empty()) {
    return 0.0;
  }

  // Find initial hint from first trajectory pose
  const geometry_msgs::msg::Pose & current_pose = traj.poses[0];
  nav2_util::PathSearchResult search_result = nav2_util::distance_from_path(
    global_path_, current_pose);

  size_t start_index = search_result.closest_segment_index;
  double total_distance = 0.0;

  // Score each trajectory pose using bidirectional search
  for (size_t traj_index = 0; traj_index < traj.poses.size(); traj_index++) {
    SegmentSearchResult result = findClosestSegmentWithLookback(
      global_path_, traj.poses[traj_index], start_index, search_window_);

    total_distance += result.distance;
    start_index = result.closest_index;  // Update hint for next iteration
  }

  return total_distance / traj.poses.size();
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathHugCritic, dwb_core::TrajectoryCritic)
