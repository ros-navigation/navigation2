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

#include "nav2_mppi_controller/critics/path_hug_critic.hpp"


namespace mppi::critics
{

void PathHugCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0f);
  getParam(trajectory_point_step_, "trajectory_point_step", 4);
  getParam(threshold_to_consider_, "threshold_to_consider", 0.5f);
  getParam(search_window_, "search_window", 0.15f);
  getParam(lookahead_distance_, "lookahead_distance", 0.3f);
  getParam(max_allowed_distance_, "max_allowed_distance", 0.2f);
  getParam(collision_cost_, "collision_cost", 100000.0f);
  getParam(use_soft_repulsion_, "use_soft_repulsion", false);
  getParam(grace_distance_, "grace_distance", 0.5f * max_allowed_distance_);
  getParam(fallback_ratio_, "fallback_ratio", 0.3f);
  getParam(recovery_weight_, "recovery_weight", 5.0f);
  getParam(min_path_point_spacing_, "min_path_point_spacing", 0.0f);

  clock_ = parent_.lock()->get_clock();

  if (max_allowed_distance_ <= 0.0f) {
    RCLCPP_WARN(
      logger_,
      "PathHugCritic: max_allowed_distance must be > 0. Clamping to 0.05 m.");
    max_allowed_distance_ = 0.05f;
  }

  if (grace_distance_ >= max_allowed_distance_) {
    RCLCPP_WARN(
      logger_,
      "PathHugCritic: grace_distance (%f) >= max_allowed_distance (%f). "
      "Clamping to 0.5 * max_allowed_distance.",
      grace_distance_, max_allowed_distance_);
    grace_distance_ = 0.5f * max_allowed_distance_;
  }

  RCLCPP_INFO(
    logger_,
    "PathHugCritic instantiated with %d power and %f weight",
    power_, weight_);
}

void PathHugCritic::buildDecimatedPath(const models::Path & path, size_t num_points)
{
  decimated_indices_.clear();
  if (num_points == 0) {
    return;
  }

  // Disabled or too few points: identity mapping (behavior identical to full-resolution).
  if (min_path_point_spacing_ <= 0.0f || num_points < 3) {
    decimated_indices_.resize(num_points);
    std::iota(decimated_indices_.begin(), decimated_indices_.end(), size_t(0));
    return;
  }

  const float min_spacing_sq = min_path_point_spacing_ * min_path_point_spacing_;
  decimated_indices_.push_back(0);  // Always keep the first point
  float last_x = path.x(0);
  float last_y = path.y(0);

  // Keep a point only once it is at least min_path_point_spacing_ from the last kept point.
  for (size_t i = 1; i + 1 < num_points; ++i) {
    const float dx = path.x(i) - last_x;
    const float dy = path.y(i) - last_y;
    if (dx * dx + dy * dy >= min_spacing_sq) {
      decimated_indices_.push_back(i);
      last_x = path.x(i);
      last_y = path.y(i);
    }
  }

  decimated_indices_.push_back(num_points - 1);  // Always keep the last point
}

void PathHugCritic::updateCumulativeDistances(const models::Path & path, size_t num_segments)
{
  if (cumulative_distances_.size() != num_segments + 1) {
    cumulative_distances_.resize(num_segments + 1);
  }

  cumulative_distances_[0] = 0.0f;
  for (size_t i = 0; i < num_segments; ++i) {
    const float dx = path.x(decimated_indices_[i + 1]) - path.x(decimated_indices_[i]);
    const float dy = path.y(decimated_indices_[i + 1]) - path.y(decimated_indices_[i]);
    cumulative_distances_[i + 1] = cumulative_distances_[i] + std::sqrt(dx * dx + dy * dy);
  }
}

float PathHugCritic::computeMinDistToPathSq(
  float px, float py,
  const models::Path & path,
  size_t num_segments,
  Eigen::Index & path_hint)
{
  if (num_segments == 0) {
    return std::numeric_limits<float>::max();
  }

  path_hint = std::clamp(path_hint, Eigen::Index(0), static_cast<Eigen::Index>(num_segments - 1));

  auto distSqToSegment = [&](size_t seg_idx) -> float {
      const float x0 = path.x(decimated_indices_[seg_idx]);
      const float y0 = path.y(decimated_indices_[seg_idx]);
      const float x1 = path.x(decimated_indices_[seg_idx + 1]);
      const float y1 = path.y(decimated_indices_[seg_idx + 1]);
      const float dx = x1 - x0;
      const float dy = y1 - y0;
      const float len_sq = dx * dx + dy * dy;

      if (len_sq < 1e-6f) {
        const float dpx = px - x0;
        const float dpy = py - y0;
        return dpx * dpx + dpy * dpy;
      }

      const float dpx = px - x0;
      const float dpy = py - y0;
      const float t = std::clamp((dpx * dx + dpy * dy) / len_sq, 0.0f, 1.0f);
      const float dist_x = dpx - t * dx;
      const float dist_y = dpy - t * dy;
      return dist_x * dist_x + dist_y * dist_y;
    };

  float min_dist_sq = distSqToSegment(path_hint);
  Eigen::Index best_seg = path_hint;

  const float hint_dist = cumulative_distances_[path_hint];
  const float min_dist_threshold = hint_dist - search_window_;
  const float max_dist_threshold = hint_dist + search_window_;

  // Exponential search backward from hint
  size_t start = static_cast<size_t>(path_hint);
  for (size_t step = 1; start > 0; step *= 2) {
    const size_t test = start > step ? start - step : 0;
    if (cumulative_distances_[test] < min_dist_threshold) {break;}
    start = test;
    if (start == 0) {break;}
  }
  if (static_cast<size_t>(path_hint) > 0) {
    start = std::min(start, static_cast<size_t>(path_hint) - 1);
  }

  // Exponential search forward from hint
  size_t end = static_cast<size_t>(path_hint);
  for (size_t step = 1; end < num_segments; step *= 2) {
    const size_t test = std::min(end + step, num_segments - 1);
    if (cumulative_distances_[test] > max_dist_threshold) {break;}
    end = test;
    if (end == num_segments - 1) {break;}
  }
  if (static_cast<size_t>(path_hint) < num_segments - 1) {
    end = std::max(end, static_cast<size_t>(path_hint) + 1);
  }

  // Linear scan within window
  for (size_t i = start; i <= end; ++i) {
    const float dist_sq = distSqToSegment(i);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      best_seg = static_cast<Eigen::Index>(i);
    }
  }

  path_hint = best_seg;
  return min_dist_sq;
}

void PathHugCritic::score(CriticData & data)
{
  if (!enabled_ || data.state.local_path_length < threshold_to_consider_) {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);
  const size_t path_segments_count = *data.furthest_reached_path_point;

  if (path_segments_count < 2) {
    return;
  }

  utils::setPathCostsIfNotSet(data, costmap_ros_);
  const std::vector<bool> & path_pts_valid = *data.path_pts_valid;
  if (path_pts_valid.empty()) {
    return;
  }

  // path_segments_count is clamped to path.x.size()-1 by findPathFurthestReachedPoint,
  // so +1 yields a valid point count never exceeding path.x.size().
  buildDecimatedPath(data.path, path_segments_count + 1);
  const size_t num_segments = decimated_indices_.size() - 1;
  updateCumulativeDistances(data.path, num_segments);

  const auto & traj_x = data.trajectories.x;
  const auto & traj_y = data.trajectories.y;
  const Eigen::Index batch_size = traj_x.rows();
  const Eigen::Index traj_length = traj_x.cols();

  const float robot_x = data.state.pose.pose.position.x;
  const float robot_y = data.state.pose.pose.position.y;

  Eigen::Index robot_hint = 0;
  computeMinDistToPathSq(robot_x, robot_y, data.path, num_segments, robot_hint);

  const int effective_stride = std::max(
    1, std::min(trajectory_point_step_, static_cast<int>(traj_length)));

  const float max_dist_sq = max_allowed_distance_ * max_allowed_distance_;
  const float grace_dist_sq = grace_distance_ * grace_distance_;
  const float repulsion_zone = max_allowed_distance_ - grace_distance_;
  const float fallback_lookahead = fallback_ratio_ * lookahead_distance_;

  if (results_.size() != static_cast<size_t>(batch_size)) {
    results_.resize(batch_size);
  }
  std::fill(results_.begin(), results_.end(), TrajResult{});

  Eigen::Index violation_count = 0;

  // Fallback cost accumulates within fallback_lookahead even for violators,
  // preserving recovery gradient if all trajectories violate.
  for (Eigen::Index traj_idx = 0; traj_idx < batch_size; ++traj_idx) {
    TrajResult & traj = results_[traj_idx];
    Eigen::Index path_hint = robot_hint;

    float accumulated_distance = 0.0f;
    float prev_x = traj_x(traj_idx, 0);
    float prev_y = traj_y(traj_idx, 0);

    for (Eigen::Index col = effective_stride; col < traj_length; col += effective_stride) {
      const float px = traj_x(traj_idx, col);
      const float py = traj_y(traj_idx, col);

      const float step_dx = px - prev_x;
      const float step_dy = py - prev_y;
      accumulated_distance += std::sqrt(step_dx * step_dx + step_dy * step_dy);
      prev_x = px;
      prev_y = py;

      if (accumulated_distance > lookahead_distance_) {break;}

      const float dist_sq = computeMinDistToPathSq(
        px, py, data.path, num_segments, path_hint);

      const size_t safe_hint = std::min(
        static_cast<size_t>(path_hint), decimated_indices_.size() - 1);
      const size_t orig_idx = std::min(
        decimated_indices_[safe_hint], path_pts_valid.size() - 1);
      if (!path_pts_valid[orig_idx]) {continue;}

      traj.num_samples++;
      const float dist = std::sqrt(dist_sq);

      if (accumulated_distance <= fallback_lookahead) {
        traj.fallback_cost += dist;
        traj.fallback_samples++;
      }

      if (dist_sq > max_dist_sq) {
        if (!traj.violates) {traj.violates = true; violation_count++;}
        if (accumulated_distance > fallback_lookahead) {break;}
        continue;
      }

      if (use_soft_repulsion_ && dist_sq > grace_dist_sq) {
        traj.repulsion_cost += (dist - grace_distance_) / repulsion_zone;
      }
    }

    if (traj.num_samples > 0 && !traj.violates) {
      traj.repulsion_cost /= static_cast<float>(traj.num_samples);
    }
  }

  const bool all_violate = (violation_count == batch_size);

  if (!all_violate) {
    for (Eigen::Index traj_idx = 0; traj_idx < batch_size; ++traj_idx) {
      const TrajResult & r = results_[traj_idx];
      if (r.violates) {
        data.costs(traj_idx) += collision_cost_;
      } else if (use_soft_repulsion_ && r.repulsion_cost > 0.0f) {
        if (power_ > 1u) {
          data.costs(traj_idx) +=
            std::pow(r.repulsion_cost * weight_, static_cast<float>(power_));
        } else {
          data.costs(traj_idx) += r.repulsion_cost * weight_;
        }
      }
    }
  } else {
    // Apply short sighted heavier weight to getin path quickly
    RCLCPP_DEBUG_THROTTLE(
      logger_, *clock_, 5000,
      "PathHugCritic: ALL %ld trajectories violate the corridor boundary "
      "(max_allowed_distance: %.3f m). Applying graded recovery.",
      static_cast<long>(batch_size), max_allowed_distance_);

    for (Eigen::Index traj_idx = 0; traj_idx < batch_size; ++traj_idx) {
      const TrajResult & r = results_[traj_idx];
      if (r.fallback_samples > 0) {
        const float avg = r.fallback_cost / static_cast<float>(r.fallback_samples);
        if (power_ > 1u) {
          data.costs(traj_idx) += std::pow(avg * recovery_weight_, static_cast<float>(power_));
        } else {
          data.costs(traj_idx) += avg * recovery_weight_;
        }
      }
    }
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathHugCritic,
  mppi::critics::CriticFunction)
