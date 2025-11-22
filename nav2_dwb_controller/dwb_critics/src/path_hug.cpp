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
    node, dwb_plugin_name_ + ".search_window", rclcpp::ParameterValue(2.0));
  node->get_parameter(dwb_plugin_name_ + ".search_window", search_window_);

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

double PathHugCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (traj.poses.empty() || global_path_.poses.empty()) {
    return 0.0;
  }
  double distance = 0;

  current_pose_ = traj.poses[0];
  nav2_util::PathSearchResult search_result = nav2_util::distance_from_path(
    global_path_, current_pose_);
  start_index_ = search_result.closest_segment_index;

  for (size_t traj_index = 0; traj_index < traj.poses.size(); traj_index++) {
    search_result = nav2_util::distance_from_path(global_path_, traj.poses[traj_index],
      start_index_,
      search_window_);
    distance += search_result.distance;
    start_index_ = search_result.closest_segment_index;
  }
  return distance / traj.poses.size();
}

double PathHugCritic::getScale() const
{
  if (zero_scale_) {
    return 0.0;
  } else {
    return costmap_ros_->getCostmap()->getResolution() * 0.5;
  }
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathHugCritic, dwb_core::TrajectoryCritic)
