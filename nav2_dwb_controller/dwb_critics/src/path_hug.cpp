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
#include "dwb_critics/alignment_util.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/path_utils.hpp"

namespace dwb_critics
{

void PathHugCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".scale", rclcpp::ParameterValue(36.0));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".scale", scale_);

  nav2::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".search_window", rclcpp::ParameterValue(1.5));
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

  // Calculate search range with lookback (bidirectional window)
  const int half_window = static_cast<int>(search_window / 2.0);
  const size_t start_idx = (hint_index > static_cast<size_t>(half_window)) ?
    hint_index - half_window :
    0;
  const size_t end_idx = std::min(path_size - 1, hint_index + half_window);

  double min_distance = std::numeric_limits<double>::max();
  size_t closest_index = hint_index;

  // Search within the window for closest segment
  for (size_t i = start_idx; i <= end_idx; ++i) {
    const double dist = nav2_util::geometry_utils::euclidean_distance(
      pose.position, path.poses[i].pose.position);

    if (dist < min_distance) {
      min_distance = dist;
      closest_index = i;
    }
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

  double distance = 0.0;

  // Score each trajectory pose using bidirectional search
  for (size_t traj_index = 0; traj_index < traj.poses.size(); traj_index++) {
    SegmentSearchResult result = findClosestSegmentWithLookback(
      global_path_, traj.poses[traj_index], start_index, search_window_);

    distance += result.distance;
    start_index = result.closest_index;  // Update hint for next iteration
  }

  return distance / traj.poses.size();
}

double PathHugCritic::getScale() const
{
  return costmap_ros_->getCostmap()->getResolution() * 0.5 * scale_;
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathHugCritic, dwb_core::TrajectoryCritic)
