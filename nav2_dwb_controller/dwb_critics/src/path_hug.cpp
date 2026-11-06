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

#include "dwb_critics/path_hug.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "tf2/utils.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace dwb_critics
{

void PathHugCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  threshold_to_consider_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".threshold_to_consider", 0.2);
  search_window_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".search_window", 0.15);
  min_path_point_spacing_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".min_path_point_spacing", 0.01);
  max_allowed_distance_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".max_allowed_distance", 0.05);
  grace_distance_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".grace_distance", 0.01);
  use_soft_repulsion_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".use_soft_repulsion", true);
  power_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".power", 1);
  heading_weight_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".heading_weight", 3.0);
  tracking_lookahead_dist_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".tracking_lookahead_dist", 0.5);
  min_traj_point_spacing_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".min_traj_point_spacing", 0.01);
  critical_cost_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".critical_cost", 100.0);

  zero_scale_ = false;
  hint_ = 0;

  if (search_window_ <= 0.0) {
    RCLCPP_WARN(
      rclcpp::get_logger("PathHugCritic"),
      "PathHugCritic: search_window must be > 0. Clamping to 1.0 m.");
    search_window_ = 1.0;
  }

  if (max_allowed_distance_ <= 0.0) {
    RCLCPP_WARN(
      rclcpp::get_logger("PathHugCritic"),
      "PathHugCritic: max_allowed_distance must be > 0. Clamping to 0.05 m.");
    max_allowed_distance_ = 0.05;
  }

  if (grace_distance_ >= max_allowed_distance_) {
    RCLCPP_WARN(
      rclcpp::get_logger("PathHugCritic"),
      "PathHugCritic: grace_distance (%f) >= max_allowed_distance (%f). "
      "Clamping to 0.5 * max_allowed_distance.",
      grace_distance_, max_allowed_distance_);
    grace_distance_ = 0.5 * max_allowed_distance_;
  }
}

bool PathHugCritic::prepare(
  const geometry_msgs::msg::Pose & pose, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose &,
  const nav_msgs::msg::Path & global_plan)
{
  if (global_plan.poses.size() < 2) {
    return false;
  }

  const double path_length = nav2_util::geometry_utils::calculate_path_length(global_plan);

  if (path_length < threshold_to_consider_) {
    zero_scale_ = true;
    return true;
  }

  zero_scale_ = false;

  global_plan_ = decimatePlan(global_plan);

  const size_t np = global_plan_.poses.size();
  cumulative_distances_.resize(np);
  cumulative_distances_[0] = 0.0;
  for (size_t i = 1; i < np; ++i) {
    cumulative_distances_[i] = cumulative_distances_[i - 1] +
      nav2_util::geometry_utils::euclidean_distance(
      global_plan_.poses[i - 1].pose.position, global_plan_.poses[i].pose.position);
  }

  // Unbounded window gives a global nearest to seed the per-trajectory hint.
  hint_ = findClosestSegment(
    global_plan_, pose, 0, std::numeric_limits<double>::max()).closest_index;

  return true;
}

nav_msgs::msg::Path PathHugCritic::decimatePlan(const nav_msgs::msg::Path & path) const
{
  if (min_path_point_spacing_ <= 0.0 || path.poses.size() < 3) {
    return path;
  }

  const double min_spacing_sq = min_path_point_spacing_ * min_path_point_spacing_;

  nav_msgs::msg::Path decimated;
  decimated.header = path.header;
  decimated.poses.reserve(path.poses.size());

  decimated.poses.push_back(path.poses.front());
  double last_x = path.poses.front().pose.position.x;
  double last_y = path.poses.front().pose.position.y;

  for (size_t i = 1; i + 1 < path.poses.size(); ++i) {  // last point kept separately below
    const double dx = path.poses[i].pose.position.x - last_x;
    const double dy = path.poses[i].pose.position.y - last_y;
    if (dx * dx + dy * dy >= min_spacing_sq) {
      decimated.poses.push_back(path.poses[i]);
      last_x = path.poses[i].pose.position.x;
      last_y = path.poses[i].pose.position.y;
    }
  }

  decimated.poses.push_back(path.poses.back());
  return decimated;
}

PathHugCritic::SegmentSearchResult PathHugCritic::findClosestSegment(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Pose & pose,
  size_t hint_index,
  double search_window) const
{
  const size_t path_size = path.poses.size();

  if (path_size == 0) {
    return {0, 0.0};
  }
  if (path_size == 1) {
    return {0, nav2_util::geometry_utils::euclidean_distance(
        pose.position, path.poses[0].pose.position)};
  }

  if (hint_index > path_size - 2) {
    hint_index = path_size - 2;
  }

  const double half_window = search_window / 2.0;
  double min_distance_sq = std::numeric_limits<double>::max();
  size_t closest_index = hint_index;

  // Backward scan: seg = i-1 so hint_index itself is left for the forward scan only.
  double traversed = 0.0;
  for (size_t i = hint_index; i > 0 && traversed <= half_window; --i) {
    const size_t seg = i - 1;
    const double dist_sq = distanceSqToSegment(
      pose.position, path.poses[seg].pose.position, path.poses[seg + 1].pose.position);
    if (dist_sq < min_distance_sq) {
      min_distance_sq = dist_sq;
      closest_index = seg;
    }
    traversed += cumulative_distances_[seg + 1] - cumulative_distances_[seg];
  }

  // Forward scan: each direction gets its own budget so the window is symmetric.
  traversed = 0.0;
  for (size_t seg = hint_index; seg + 1 < path_size && traversed <= half_window; ++seg) {
    const double dist_sq = distanceSqToSegment(
      pose.position, path.poses[seg].pose.position, path.poses[seg + 1].pose.position);
    if (dist_sq < min_distance_sq) {
      min_distance_sq = dist_sq;
      closest_index = seg;
    }
    traversed += cumulative_distances_[seg + 1] - cumulative_distances_[seg];
  }

  return {closest_index, std::sqrt(min_distance_sq)};
}

double PathHugCritic::getScale() const
{
  return zero_scale_ ? 0.0 : scale_;
}

double PathHugCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  const size_t n = traj.poses.size();
  if (n == 0 || global_plan_.poses.size() < 2) {
    return 0.0;
  }

  const double repulsion_zone = max_allowed_distance_ - grace_distance_;

  size_t hint = hint_;

  double total_cost = 0.0;
  size_t scored_poses = 0;
  double traversed_dist = 0.0;
  double last_scored_x = traj.poses[0].position.x;
  double last_scored_y = traj.poses[0].position.y;
  const double min_spacing_sq = min_traj_point_spacing_ * min_traj_point_spacing_;

  auto scorePose = [&](const geometry_msgs::msg::Pose & pose) {
      const SegmentSearchResult result = findClosestSegment(
      global_plan_, pose, hint, search_window_);
      hint = result.closest_index;

      const double dist = result.distance;

      double point_cost;
      if (dist <= grace_distance_) {
        point_cost = 0.0;
      } else if (dist <= max_allowed_distance_) {
        point_cost = use_soft_repulsion_ ?
          (dist - grace_distance_) / repulsion_zone : 0.0;
      } else {
      // +1.0 ensures continuity with the repulsion band at its boundary (1.0).
        point_cost = critical_cost_ + 1.0 + (dist - max_allowed_distance_) / repulsion_zone;
      }

      if (power_ > 1 && point_cost > 0.0) {
        point_cost = std::pow(point_cost, static_cast<double>(power_));
      }

      if (heading_weight_ > 0.0) {
        const size_t seg = result.closest_index;
        const auto & pa = global_plan_.poses[seg].pose.position;
        const auto & pb = global_plan_.poses[seg + 1].pose.position;
        const double dx = pb.x - pa.x;
        const double dy = pb.y - pa.y;
        if (dx * dx + dy * dy > kHeadingEps * kHeadingEps) {
          const double path_yaw = std::atan2(dy, dx);
          const double robot_yaw = tf2::getYaw(pose.orientation);
          double heading_cost =
            std::fabs(angles::shortest_angular_distance(robot_yaw, path_yaw)) / M_PI;
          if (power_ > 1 && heading_cost > 0.0) {
            heading_cost = std::pow(heading_cost, static_cast<double>(power_));
          }
          point_cost += heading_weight_ * heading_cost;
        }
      }

      total_cost += point_cost;
      ++scored_poses;
    };

  // Always score first and last to guarantee at least two anchor points.
  scorePose(traj.poses.front());
  last_scored_x = traj.poses.front().position.x;
  last_scored_y = traj.poses.front().position.y;

  bool truncated = false;
  for (size_t i = 1; i + 1 < n; ++i) {
    const auto & pose = traj.poses[i];

    // Accumulates traversed distance to limit scoring to tracking_lookahead_dist_.
    if (tracking_lookahead_dist_ > 0.0) {
      const double dx = pose.position.x - traj.poses[i - 1].position.x;
      const double dy = pose.position.y - traj.poses[i - 1].position.y;
      traversed_dist += std::sqrt(dx * dx + dy * dy);
      if (traversed_dist > tracking_lookahead_dist_) {
        truncated = true;
        break;
      }
    }

    if (min_traj_point_spacing_ > 0.0) {
      const double dx = pose.position.x - last_scored_x;
      const double dy = pose.position.y - last_scored_y;
      if (dx * dx + dy * dy < min_spacing_sq) {
        continue;
      }
    }

    scorePose(pose);
    last_scored_x = pose.position.x;
    last_scored_y = pose.position.y;
  }

  // Skip the endpoint if truncated: its position would corrupt the mean of a partial prefix.
  if (n > 1 && !truncated) {
    scorePose(traj.poses.back());
  }

  return total_cost / static_cast<double>(scored_poses);
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathHugCritic, dwb_core::TrajectoryCritic)
