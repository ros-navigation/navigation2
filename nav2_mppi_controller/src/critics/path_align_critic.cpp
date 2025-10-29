// Copyright (c) 2023 Open Navigation LLC
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

#include "nav2_mppi_controller/critics/path_align_critic.hpp"

namespace mppi::critics
{

void PathAlignCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0f);
  getParam(max_path_occupancy_ratio_, "max_path_occupancy_ratio", 0.07f);
  getParam(offset_from_furthest_, "offset_from_furthest", 20);
  getParam(trajectory_point_step_, "trajectory_point_step", 4);
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.5f);
  getParam(use_path_orientations_, "use_path_orientations", false);
  getParam(use_geometric_alignment_,"use_geometric_alignment");
  getParam(search_window_,"search_window")

  RCLCPP_INFO(
    logger_,
    "ReferenceTrajectoryCritic instantiated with %d power and %f weight",
    power_, weight_);
}

void PathAlignCritic::score(CriticData & data){
  if (!enabled_ || data.state.local_path_length < threshold_to_consider_) {
    return;
  }

  // Don't apply when first getting bearing w.r.t. the path
  utils::setPathFurthestPointIfNotSet(data);
  // Up to furthest only, closest path point is always 0 from path handler
  const size_t path_segments_count = *data.furthest_reached_path_point;
  float path_segments_flt = static_cast<float>(path_segments_count);
  if (path_segments_count < offset_from_furthest_) {
    return;
  }

  // Don't apply when dynamic obstacles are blocking significant proportions of the local path
  utils::setPathCostsIfNotSet(data, costmap_ros_);
  std::vector<bool> & path_pts_valid = *data.path_pts_valid;
  float invalid_ctr = 0.0f;
  for (size_t i = 0; i < path_segments_count; i++) {
    if (!path_pts_valid[i]) {invalid_ctr += 1.0f;}
    if (invalid_ctr / path_segments_flt > max_path_occupancy_ratio_ && invalid_ctr > 2.0f) {
      return;
    }
  }
  if (use_geometric_alignment_) {
    scoreGeometric(data, path_pts_valid);
  } else {
    scoreArcLength(data, path_pts_valid);
  }
}

void PathAlignCritic::scoreArcLength(CriticData & data, std::vector<bool> & path_pts_valid)
{
  // Original PathAlignCritic implementation
  const size_t path_segments_count = *data.furthest_reached_path_point;
  const size_t batch_size = data.trajectories.x.rows();
  Eigen::ArrayXf cost(data.costs.rows());
  cost.setZero();

  std::vector<float> path_integrated_distances(path_segments_count, 0.0f);
  std::vector<utils::Pose2D> path(path_segments_count);
  float dx = 0.0f, dy = 0.0f;
  
  for (unsigned int i = 1; i != path_segments_count; i++) {
    auto & pose = path[i - 1];
    pose.x = data.path.x(i - 1);
    pose.y = data.path.y(i - 1);
    pose.theta = data.path.yaws(i - 1);

    dx = data.path.x(i) - pose.x;
    dy = data.path.y(i) - pose.y;
    path_integrated_distances[i] = path_integrated_distances[i - 1] + sqrtf(dx * dx + dy * dy);
  }

  auto & final_pose = path[path_segments_count - 1];
  final_pose.x = data.path.x(path_segments_count - 1);
  final_pose.y = data.path.y(path_segments_count - 1);
  final_pose.theta = data.path.yaws(path_segments_count - 1);

  float summed_path_dist = 0.0f, dyaw = 0.0f;
  unsigned int num_samples = 0u;
  unsigned int path_pt = 0u;
  float traj_integrated_distance = 0.0f;

  int strided_traj_rows = data.trajectories.x.rows();
  int strided_traj_cols = floor((data.trajectories.x.cols() - 1) / trajectory_point_step_) + 1;
  int outer_stride = strided_traj_rows * trajectory_point_step_;
  
  const auto T_x = Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
    data.trajectories.x.data(), strided_traj_rows, strided_traj_cols, 
    Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto T_y = Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
    data.trajectories.y.data(), strided_traj_rows, strided_traj_cols, 
    Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto T_yaw = Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
    data.trajectories.yaws.data(), strided_traj_rows, strided_traj_cols,
    Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto traj_sampled_size = T_x.cols();

  for (size_t t = 0; t < batch_size; ++t) {
    summed_path_dist = 0.0f;
    num_samples = 0u;
    traj_integrated_distance = 0.0f;
    path_pt = 0u;
    float Tx_m1 = T_x(t, 0);
    float Ty_m1 = T_y(t, 0);
    
    for (int p = 1; p < traj_sampled_size; p++) {
      const float Tx = T_x(t, p);
      const float Ty = T_y(t, p);
      dx = Tx - Tx_m1;
      dy = Ty - Ty_m1;
      Tx_m1 = Tx;
      Ty_m1 = Ty;
      traj_integrated_distance += sqrtf(dx * dx + dy * dy);
      path_pt = utils::findClosestPathPt(path_integrated_distances, traj_integrated_distance, path_pt);

      if (path_pts_valid[path_pt]) {
        const auto & pose = path[path_pt];
        dx = pose.x - Tx;
        dy = pose.y - Ty;
        num_samples++;
        if (use_path_orientations_) {
          dyaw = angles::shortest_angular_distance(pose.theta, T_yaw(t, p));
          summed_path_dist += sqrtf(dx * dx + dy * dy + dyaw * dyaw);
        } else {
          summed_path_dist += sqrtf(dx * dx + dy * dy);
        }
      }
    }
    cost(t) = (num_samples > 0u) ? summed_path_dist / static_cast<float>(num_samples) : 0.0f;
  }

  if (power_ > 1u) {
    data.costs += (cost * weight_).pow(power_).eval();
  } else {
    data.costs += (cost * weight_).eval();
  }
}

void PathAlignCritic::scoreGeometric(CriticData & data, std::vector<bool> & path_pts_valid)
{
  // PathHugCritic geometric approach with obstacle awareness
  if (static_cast<Eigen::Index>(data.path.x.size()) != static_cast<Eigen::Index>(path_size_cache_) ||
      (data.path.x.size() > 0 && path_x_cache_.size() == data.path.x.size() &&
       (data.path.x.array() != path_x_cache_.array()).any()))
  {
    updatePathCache(data.path);
  }

  const auto & traj_x = data.trajectories.x;
  const auto & traj_y = data.trajectories.y;
  const Eigen::Index batch_size = traj_x.rows();
  const Eigen::Index traj_length = traj_x.cols();

  Eigen::ArrayXf cost_array(batch_size);
  cost_array.setZero();
  
  if (closest_indices_.size() != batch_size) {
    closest_indices_.resize(batch_size);
    closest_indices_.setZero();
  }

  const int effective_stride = std::max(1, std::min(trajectory_point_step_, static_cast<int>(traj_length)));
  const int num_samples = (traj_length + effective_stride - 1) / effective_stride;

  if (num_samples == 0) {return;}

  const size_t path_segments_count = *data.furthest_reached_path_point;

  for (int sample_idx = 0; sample_idx < num_samples; ++sample_idx) {
    const Eigen::Index traj_col = sample_idx * effective_stride;
    if (traj_col >= traj_length) {break;}

    for (Eigen::Index traj_idx = 0; traj_idx < batch_size; ++traj_idx) {
      const float px = traj_x(traj_idx, traj_col);
      const float py = traj_y(traj_idx, traj_col);
      
      Eigen::Index closest_seg = closest_indices_(traj_idx);
      float min_dist = computeMinDistanceToPath(px, py, closest_seg);
      
      // Only accumulate cost if the closest path segment is valid (no obstacle)
      if (closest_seg < static_cast<Eigen::Index>(path_segments_count) && 
          path_pts_valid[closest_seg]) {
        cost_array(traj_idx) += min_dist;
      }
      
      closest_indices_(traj_idx) = closest_seg;
    }
  }

  cost_array /= static_cast<float>(num_samples);

  if (power_ > 1u) {
    data.costs += (cost_array * weight_).pow(power_);
  } else {
    data.costs += cost_array * weight_;
  }
}

void PathAlignCritic::updatePathCache(const models::Path & path)
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

float PathAlignCritic::computeMinDistanceToPath(float px, float py, Eigen::Index & closest_seg_idx)
{
  const Eigen::Index path_size = path_x_cache_.size();
  float min_dist_sq = std::numeric_limits<float>::max();
  
  Eigen::Index start_idx = std::max(0L, closest_seg_idx - 2);
  double distance_traversed = 0.0;
  Eigen::Index best_seg = start_idx;

  // Search window optimization
  for (Eigen::Index seg_idx = start_idx; seg_idx < path_size - 1; ++seg_idx) {
    if (distance_traversed > search_window_) {break;}
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
      best_seg = seg_idx;
    }
    distance_traversed += segment_lengths_(seg_idx);
  }

  // Fallback: full search if window failed
  if (min_dist_sq == std::numeric_limits<float>::max()) {
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
        best_seg = seg_idx;
      }
    }
  }

  closest_seg_idx = best_seg;
  return std::sqrt(min_dist_sq);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAlignCritic,
  mppi::critics::CriticFunction)
