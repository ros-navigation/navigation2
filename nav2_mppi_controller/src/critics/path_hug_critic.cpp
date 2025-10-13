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

#include "nav2_mppi_controller/critics/path_hug_critic.hpp"

#include <Eigen/Dense>
#include <math.h>
#include <limits>

#include "nav2_util/geometry_utils.hpp"

namespace mppi::critics
{

void PathHugCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0f);
  getParam(search_window_, "search_window", 2.0);
  getParam(sample_stride_, "sample_stride", 3);

  RCLCPP_INFO(
    logger_,
    "PathHugCritic instantiated with power=%u, weight=%.2f, search_window=%.2f, stride=%d",
    power_, weight_, search_window_, sample_stride_);
}

void PathHugCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }
  if (data.path.x.size() < 2) {
    return;
  }

  if (static_cast<Eigen::Index>(data.path.x.size()) !=
    static_cast<Eigen::Index>(path_size_cache_) ||
    (data.path.x.size() > 0 && path_x_cache_.size() == data.path.x.size() &&
    (data.path.x.array() != path_x_cache_.array()).any()))
  {
    updatePathCache(data.path);
  }

  const auto & traj_x = data.trajectories.x;
  const auto & traj_y = data.trajectories.y;
  const Eigen::Index batch_size = traj_x.rows();
  const Eigen::Index traj_length = traj_x.cols();

  // Pre-allocate arrays
  if (cost_array_.size() != batch_size) {
    cost_array_.resize(batch_size);
    closest_indices_.resize(batch_size);
    closest_indices_.setZero();
  }
  cost_array_.setZero();

  // Sample trajectory points with stride
  const int effective_stride = std::max(1, std::min(sample_stride_, static_cast<int>(traj_length)));
  const int num_samples = (traj_length + effective_stride - 1) / effective_stride;

  if (num_samples == 0) {return;}

  for (int sample_idx = 0; sample_idx < num_samples; ++sample_idx) {
    const Eigen::Index traj_col = sample_idx * effective_stride;
    if (traj_col >= traj_length) {break;}

    // Get all trajectory points at that step
    const auto & points_x = traj_x.col(traj_col);
    const auto & points_y = traj_y.col(traj_col);

    computeDistancesToPathVectorized(points_x, points_y, cost_array_);
  }

  // Normalize by the number of samples
  cost_array_ /= static_cast<float>(num_samples);

  if (power_ > 1u) {
    data.costs += (cost_array_ * weight_).pow(power_);
  } else {
    data.costs += cost_array_ * weight_;
  }
}

void PathHugCritic::updatePathCache(const models::Path & path)
{
  path_size_cache_ = path.x.size();
  path_x_cache_ = path.x;
  path_y_cache_ = path.y;

  const Eigen::Index path_size = path.x.size();
  if (path_size < 2) {return;}

  if (segment_lengths_.size() != path_size - 1) {
    segment_lengths_.resize(path_size - 1);
    cumulative_distances_.resize(path_size);
    segment_dx_.resize(path_size - 1);
    segment_dy_.resize(path_size - 1);
    segment_len_sq_.resize(path_size - 1);
  }

  cumulative_distances_(0) = 0.0;
  for (Eigen::Index i = 0; i < path_size - 1; ++i) {
    segment_dx_(i) = path.x(i + 1) - path.x(i);
    segment_dy_(i) = path.y(i + 1) - path.y(i);
    segment_len_sq_(i) = segment_dx_(i) * segment_dx_(i) + segment_dy_(i) * segment_dy_(i);
    segment_lengths_(i) = std::sqrt(segment_len_sq_(i));
    cumulative_distances_(i + 1) = cumulative_distances_(i) + segment_lengths_(i);
  }
}

void PathHugCritic::computeDistancesToPathVectorized(
  const Eigen::ArrayXf & points_x,
  const Eigen::ArrayXf & points_y,
  Eigen::ArrayXf & distances)
{
  const Eigen::Index batch_size = points_x.size();
  const Eigen::Index path_size = path_x_cache_.size();

  for (Eigen::Index traj_idx = 0; traj_idx < batch_size; ++traj_idx) {
    const float px = points_x(traj_idx);
    const float py = points_y(traj_idx);

    float min_dist_sq = std::numeric_limits<float>::max();

    Eigen::Index start_idx = (traj_idx < closest_indices_.size()) ?
      std::max(0L, closest_indices_(traj_idx) - 2) : 0;

    double distance_traversed = 0.0;
    Eigen::Index closest_seg_idx = start_idx;

    for (Eigen::Index seg_idx = start_idx; seg_idx < path_size - 1; ++seg_idx) {
      if (distance_traversed > search_window_) {
        break;
      }

      if (segment_len_sq_(seg_idx) < 1e-6) {
        continue;
      }

      // Standard point-to-line-segment distance calculation
      const float dx_to_start = px - path_x_cache_(seg_idx);
      const float dy_to_start = py - path_y_cache_(seg_idx);
      const float dot = dx_to_start * segment_dx_(seg_idx) + dy_to_start * segment_dy_(seg_idx);
      const float t = std::clamp(dot / segment_len_sq_(seg_idx), 0.0f, 1.0f);
      const float proj_x = path_x_cache_(seg_idx) + t * segment_dx_(seg_idx);
      const float proj_y = path_y_cache_(seg_idx) + t * segment_dy_(seg_idx);
      const float dist_sq = (px - proj_x) * (px - proj_x) + (py - proj_y) * (py - proj_y);

      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        closest_seg_idx = seg_idx;
      }

      distance_traversed += segment_lengths_(seg_idx);
    }

    float final_min_dist = std::sqrt(min_dist_sq);

    // Fallback if the search window found nothing
    if (min_dist_sq == std::numeric_limits<float>::max()) {
      // If search window fails, do a full search using the same accurate logic
      for (Eigen::Index seg_idx = 0; seg_idx < path_size - 1; ++seg_idx) {
        if (segment_len_sq_(seg_idx) < 1e-6) {continue;}

        const float dx_to_start = px - path_x_cache_(seg_idx);
        const float dy_to_start = py - path_y_cache_(seg_idx);
        const float dot = dx_to_start * segment_dx_(seg_idx) + dy_to_start * segment_dy_(seg_idx);
        const float t = std::clamp(dot / segment_len_sq_(seg_idx), 0.0f, 1.0f);
        const float proj_x = path_x_cache_(seg_idx) + t * segment_dx_(seg_idx);
        const float proj_y = path_y_cache_(seg_idx) + t * segment_dy_(seg_idx);
        const float dist_sq = (px - proj_x) * (px - proj_x) + (py - proj_y) * (py - proj_y);

        if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          closest_seg_idx = seg_idx;
        }
      }
      // Recalculate the final distance from the true minimum squared distance
      final_min_dist = std::sqrt(min_dist_sq);
    }

    distances(traj_idx) += final_min_dist;
    if (traj_idx < closest_indices_.size()) {
      closest_indices_(traj_idx) = closest_seg_idx;
    }
  }
}
}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathHugCritic,
  mppi::critics::CriticFunction)
