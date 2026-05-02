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
    "PathHugCritic instantiated with %d power, %f weight, search_window: %f m, "
    "lookahead: %f m, max_allowed: %f m, collision_cost: %f, "
    "use_soft_repulsion: %s, grace_distance: %f m",
    power_, weight_, search_window_, lookahead_distance_, max_allowed_distance_,
    collision_cost_, use_soft_repulsion_ ? "true" : "false", grace_distance_);
}

void PathHugCritic::updateCumulativeDistances(const models::Path & path, size_t num_segments)
{
  if (cumulative_distances_.size() != num_segments + 1) {
    cumulative_distances_.resize(num_segments + 1);
  }

  cumulative_distances_[0] = 0.0f;
  for (size_t i = 0; i < num_segments; ++i) {
    const float dx = path.x(i + 1) - path.x(i);
    const float dy = path.y(i + 1) - path.y(i);
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
    const float x0 = path.x(seg_idx);
    const float y0 = path.y(seg_idx);
    const float x1 = path.x(seg_idx + 1);
    const float y1 = path.y(seg_idx + 1);
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
    if (static_cast<Eigen::Index>(i) == path_hint) {continue;}
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

  const size_t num_segments = path_segments_count - 1;
  updateCumulativeDistances(data.path, num_segments);

  const auto & traj_x = data.trajectories.x;
  const auto & traj_y = data.trajectories.y;
  const Eigen::Index batch_size = traj_x.rows();
  const Eigen::Index traj_length = traj_x.cols();

  const float robot_x = data.state.pose.pose.position.x;
  const float robot_y = data.state.pose.pose.position.y;

  // Discard return value — call only to initialize robot_hint to the closest segment index
  Eigen::Index robot_hint = 0;
  computeMinDistToPathSq(robot_x, robot_y, data.path, num_segments, robot_hint);

  const int effective_stride = std::max(
    1, std::min(trajectory_point_step_, static_cast<int>(traj_length)));

  const float max_dist_sq = max_allowed_distance_ * max_allowed_distance_;
  const float grace_dist_sq = grace_distance_ * grace_distance_;
  const float repulsion_zone = max_allowed_distance_ - grace_distance_;

  struct TrajResult
  {
    bool violates{false};
    float repulsion_cost{0.0f};
    float excess_cost{0.0f};
    int num_samples{0};
    int excess_samples{0};
  };

  std::vector<TrajResult> results(batch_size);

  // First pass: evaluate every trajectory
  for (Eigen::Index traj_idx = 0; traj_idx < batch_size; ++traj_idx) {
    TrajResult & r = results[traj_idx];
    Eigen::Index path_hint = robot_hint;

    float accumulated_distance = 0.0f;
    float prev_x = traj_x(traj_idx, 0);
    float prev_y = traj_y(traj_idx, 0);

    for (Eigen::Index col = effective_stride; col < traj_length; col += effective_stride) {
      const float px = traj_x(traj_idx, col);
      const float py = traj_y(traj_idx, col);

      accumulated_distance += std::hypot(px - prev_x, py - prev_y);

      if (accumulated_distance > lookahead_distance_) {break;}

      const float dist_sq = computeMinDistToPathSq(
        px, py, data.path, num_segments, path_hint);

      const size_t safe_hint = std::min(
        static_cast<size_t>(path_hint), path_pts_valid.size() - 1);
      if (!path_pts_valid[safe_hint]) {
        prev_x = px;
        prev_y = py;
        continue;
      }

      r.num_samples++;

      if (dist_sq > max_dist_sq) {
        // Do not break — accumulate all excess points for accurate fallback gradient.
        r.violates = true;
        r.excess_cost += std::sqrt(dist_sq) - max_allowed_distance_;
        r.excess_samples++;
        prev_x = px;
        prev_y = py;
        continue;
      }

      if (use_soft_repulsion_ && dist_sq > grace_dist_sq) {
        const float dist = std::sqrt(dist_sq);
        r.repulsion_cost += (dist - grace_distance_) / repulsion_zone;
      }

      prev_x = px;
      prev_y = py;
    }

    if (r.num_samples > 0) {
      r.repulsion_cost /= static_cast<float>(r.num_samples);
    }
  }

  Eigen::Index violating_trajectory_count = 0;
  for (Eigen::Index i = 0; i < batch_size; ++i) {
    if (results[i].violates) {violating_trajectory_count++;}
  }

  const bool all_violate = (violating_trajectory_count == batch_size);

  if (all_violate) {
    RCLCPP_WARN_THROTTLE(
      logger_, *parent_.lock()->get_clock(), 5000,
      "PathHugCritic: ALL %ld trajectories violate the corridor boundary "
      "(max_allowed_distance: %.3f m). Applying graded fallback costs.",
      static_cast<long>(batch_size), max_allowed_distance_);
  }

  // Second pass: apply costs
  for (Eigen::Index traj_idx = 0; traj_idx < batch_size; ++traj_idx) {
    const TrajResult & r = results[traj_idx];

    if (r.violates) {
      if (all_violate) {
        // Every trajectory is outside the corridor — apply graded fallback
        const float avg_excess = r.excess_samples > 0 ?
          r.excess_cost / static_cast<float>(r.excess_samples) : r.excess_cost;
        if (power_ > 1u) {
          data.costs(traj_idx) +=
            std::pow(avg_excess * weight_, static_cast<float>(power_));
        } else {
          data.costs(traj_idx) += avg_excess * weight_;
        }
      } else {
        // Only some trajectories violate — hard veto to push the optimizer toward valid ones.
        data.costs(traj_idx) += collision_cost_;
      }
    } else if (use_soft_repulsion_ && r.repulsion_cost > 0.0f) {
      if (power_ > 1u) {
        data.costs(traj_idx) +=
          std::pow(r.repulsion_cost * weight_, static_cast<float>(power_));
      } else {
        data.costs(traj_idx) += r.repulsion_cost * weight_;
      }
    }
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathHugCritic,
  mppi::critics::CriticFunction)
