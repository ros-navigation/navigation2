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
  getParam(threshold_to_consider_, "threshold_to_consider", 0.5f);
  getParam(use_path_orientations_, "use_path_orientations", false);
  getParam(use_geometric_alignment_, "use_geometric_alignment", false);
  getParam(search_window_, "search_window", 2.0);
  getParam(score_arc_length_, "score_arc_length", true);
  getParam(lookahead_distance_, "lookahead_distance", 5.0f);

  if (!score_arc_length_ && !use_geometric_alignment_) {
    RCLCPP_WARN(logger_,
       "PathAlignCritic: Both scoring modes disabled! Critic will not function.");
  }

  std::string mode = use_geometric_alignment_ && score_arc_length_ ? "both" :
    use_geometric_alignment_ ? "geometric" :
    score_arc_length_ ? "arc-length" : "none";

  RCLCPP_INFO(
    logger_,
    "PathAlignCritic instantiated with %d power, %f weight, mode: %s",
    power_, weight_, mode.c_str());
}

void PathAlignCritic::score(CriticData & data)
{
  // Skip scoring if disabled or path too short
  if (!enabled_ || data.state.local_path_length < threshold_to_consider_) {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);
  const size_t path_segments_count = *data.furthest_reached_path_point;
  if (data.path.x.size() == 0 || path_segments_count == 0) {
    return;
  }
  if (path_segments_count < offset_from_furthest_) {
    return;
  }

  utils::setPathCostsIfNotSet(data, costmap_ros_);
  std::vector<bool> & path_pts_valid = *data.path_pts_valid;

  // Check if too many path points are in collision/invalid
  float invalid_ctr = 0.0f;
  for (size_t i = 0; i < path_segments_count; i++) {
    if (!path_pts_valid[i]) {invalid_ctr += 1.0f;}
    if (invalid_ctr / static_cast<float>(path_segments_count) > max_path_occupancy_ratio_ &&
      invalid_ctr > 2.0f)
    {
      return;  // Too many obstacles on path, don't score
    }
  }

  // Cache path segment geometry for efficient distance calculations
  updatePathCache(data.path, path_segments_count);

  // Apply selected scoring mode(s)
  if (use_geometric_alignment_) {
    scoreGeometric(data, path_pts_valid);
  }
  if (score_arc_length_) {
    scoreArcLength(data, path_pts_valid);
  }
}

void PathAlignCritic::scoreArcLength(CriticData & data, std::vector<bool> & path_pts_valid)
{
  const size_t path_segments_count = *data.furthest_reached_path_point;
  const size_t batch_size = data.trajectories.x.rows();
  Eigen::ArrayXf cost(batch_size);
  cost.setZero();

  // Build path poses for orientation matching
  std::vector<utils::Pose2D> path(path_segments_count);
  for (unsigned int i = 0; i < path_segments_count; ++i) {
    path[i].x = data.path.x(i);
    path[i].y = data.path.y(i);
    path[i].theta = data.path.yaws(i);
  }

  float summed_path_dist = 0.0f, dyaw = 0.0f, dx = 0.0f, dy = 0.0f;
  unsigned int num_samples = 0u;
  unsigned int path_pt = 0u;
  float traj_integrated_distance = 0.0f;

  const Eigen::Index traj_length = data.trajectories.x.cols();
  const int effective_stride = std::max(1, std::min(trajectory_point_step_,
    static_cast<int>(traj_length)));
  const int num_sample_points = (traj_length + effective_stride - 1) / effective_stride;

  // Score each trajectory in the batch
  for (size_t t = 0; t < batch_size; ++t) {
    summed_path_dist = 0.0f;
    num_samples = 0u;
    traj_integrated_distance = 0.0f;
    path_pt = 0u;

    // Start from first trajectory point (close to current pose, one time step ahead)
    float prev_x = data.trajectories.x(t, 0);
    float prev_y = data.trajectories.y(t, 0);

    // Sample trajectory points at regular intervals
    for (int sample_idx = 1; sample_idx < num_sample_points; ++sample_idx) {
      const Eigen::Index traj_col = sample_idx * effective_stride;
      if (traj_col >= traj_length) {break;}

      const float curr_x = data.trajectories.x(t, traj_col);
      const float curr_y = data.trajectories.y(t, traj_col);

      // Accumulate distance traveled along trajectory
      dx = curr_x - prev_x;
      dy = curr_y - prev_y;
      traj_integrated_distance += sqrtf(dx * dx + dy * dy);

      // Find corresponding path point at same arc length distance
      path_pt = utils::findClosestPathPtBinary(
        cumulative_distances_.data(), path_segments_count,
        traj_integrated_distance);

      // Calculate distance from trajectory point to corresponding path point
      if (path_pts_valid[path_pt]) {
        const auto & pose = path[path_pt];
        dx = pose.x - curr_x;
        dy = pose.y - curr_y;
        num_samples++;
        if (use_path_orientations_) {
          dyaw = angles::shortest_angular_distance(pose.theta, data.trajectories.yaws(t, traj_col));
          summed_path_dist += sqrtf(dx * dx + dy * dy + dyaw * dyaw);
        } else {
          summed_path_dist += sqrtf(dx * dx + dy * dy);
        }
      }

      prev_x = curr_x;
      prev_y = curr_y;
    }

    // Average distance across all sampled points
    cost(t) = (num_samples > 0u) ? summed_path_dist / static_cast<float>(num_samples) : 0.0f;
  }

  // Apply weight and power, then add to total costs
  if (power_ > 1u) {
    data.costs += (cost * weight_).pow(power_);
  } else {
    data.costs += cost * weight_;
  }
}

void PathAlignCritic::scoreGeometric(CriticData & data, std::vector<bool> & path_pts_valid)
{
  const auto & traj_x = data.trajectories.x;
  const auto & traj_y = data.trajectories.y;
  const Eigen::Index batch_size = traj_x.rows();
  const Eigen::Index traj_length = traj_x.cols();
  if (traj_length == 0) {
    return;
  }
  Eigen::ArrayXf cost_array(batch_size);
  cost_array.setZero();

  Eigen::ArrayXi valid_sample_count(batch_size);
  valid_sample_count.setZero();

  // Store closest segment index per trajectory for search efficiency
  if (closest_indices_.size() != batch_size) {
    closest_indices_.resize(batch_size);
    closest_indices_.setZero();
  }

  // Reset hints to robot's current position on path to avoid stale hints from previous cycle
  const float robot_x = data.state.pose.pose.position.x;
  const float robot_y = data.state.pose.pose.position.y;
  const size_t path_segments_count = *data.furthest_reached_path_point;
  const Eigen::Index num_segments = static_cast<Eigen::Index>(path_size_cache_) - 1;
  
  Eigen::Index current_closest_seg = 0;
  float min_dist_sq = std::numeric_limits<float>::max();
  
  // Find initial hint by locating robot position on path
  for (Eigen::Index i = 0; i < num_segments && i < static_cast<Eigen::Index>(path_segments_count); ++i) {
    const float dist_sq = distSqToSegment(robot_x, robot_y, i);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      current_closest_seg = i;
    }
  }
  
  closest_indices_.setConstant(current_closest_seg);

  const int effective_stride = std::max(1, std::min(trajectory_point_step_,
     static_cast<int>(traj_length)));

  // Score each trajectory in the batch
  for (Eigen::Index traj_idx = 0; traj_idx < batch_size; ++traj_idx) {
    float accumulated_distance = 0.0f;
    
    // Start from first trajectory point
    float prev_x = traj_x(traj_idx, 0);
    float prev_y = traj_y(traj_idx, 0);
    
    int sample_idx = 0;
    
    // Sample trajectory points at regular intervals
    while (true) {
      const Eigen::Index traj_col = sample_idx * effective_stride;
      if (traj_col >= traj_length) {break;}

      const float px = traj_x(traj_idx, traj_col);
      const float py = traj_y(traj_idx, traj_col);

      // Accumulate distance traveled along trajectory
      if (sample_idx > 0) {
        const float dx = px - prev_x;
        const float dy = py - prev_y;
        accumulated_distance += std::sqrt(dx * dx + dy * dy);
        
        // Stop scoring beyond lookahead distance
        if (accumulated_distance > lookahead_distance_) {break;}
      }

      // Use previous closest segment as hint for efficient search
      Eigen::Index closest_seg = closest_indices_(traj_idx);
      float min_dist = computeMinDistanceToPath(px, py, closest_seg);

      // Only accumulate cost if the closest segment is valid
      if (closest_seg < static_cast<Eigen::Index>(path_segments_count) &&
        path_pts_valid[closest_seg])
      {
        cost_array(traj_idx) += min_dist;
        valid_sample_count(traj_idx)++;
      }

      // Update hint for next sample
      closest_indices_(traj_idx) = closest_seg;
      
      prev_x = px;
      prev_y = py;
      sample_idx++;
    }
  }

  // Normalize by sampled trajectory length to make cost independent of sampling rate
  for (Eigen::Index i = 0; i < batch_size; ++i) {
    if (valid_sample_count(i) > 0) {
      const float sampled_length = static_cast<float>(valid_sample_count(i) * effective_stride);
      cost_array(i) /= sampled_length;
    }
  }

  // Apply weight and power, then add to total costs
  if (power_ > 1u) {
    data.costs += (cost_array * weight_).pow(power_);
  } else {
    data.costs += cost_array * weight_;
  }
}

void PathAlignCritic::updatePathCache(const models::Path & path, size_t path_segments_count)
{
  // Handle empty path
  if (path_segments_count == 0) {
    path_size_cache_ = 0;
    return;
  }

  // path_segments_count is the number of segments, so number of points is segments + 1
  const size_t path_points_count = path_segments_count + 1;

  // Only update cache if path changed
  // TODO: Optimize path change detection. Current element-wise comparison is expensive.
  // Possible approaches:
  //   1. Add hash field to models::Path (cleanest, needs controller-level change)
  //   2. Compute local hash here (self-contained, small overhead)
  //   3. Size-only check (fast but may miss updates when path replanned with same length)
  //   4. Spot-check key points (middle, endpoints - may still miss changes)
  // Seeking feedback on preferred approach.
  const Eigen::Index path_points_idx = static_cast<Eigen::Index>(path_points_count);
  if (path_points_idx == static_cast<Eigen::Index>(path_size_cache_) &&
    path_x_cache_.size() == path_points_idx &&
    (path.x.head(path_points_idx).array() == path_x_cache_.head(path_points_idx).array()).all())
  {
    return;  // Path unchanged, use cached values
  }

  path_size_cache_ = path_points_count;

  if (static_cast<Eigen::Index>(path_x_cache_.size()) != path_points_idx) {
    path_x_cache_.resize(path_points_idx);
    path_y_cache_.resize(path_points_idx);
  }

  // Cache path points
  path_x_cache_.head(path_points_idx) = path.x.head(path_points_idx);
  path_y_cache_.head(path_points_idx) = path.y.head(path_points_idx);

  if (path_points_count < 2) {return;}

  const Eigen::Index num_segments = path_points_idx - 1;

  if (static_cast<Eigen::Index>(segment_lengths_.size()) != num_segments) {
    segment_lengths_.resize(num_segments);
    cumulative_distances_.resize(path_points_idx);
    segment_dx_.resize(num_segments);
    segment_dy_.resize(num_segments);
    segment_len_sq_.resize(num_segments);
  }

  // Precompute segment geometry for efficient distance calculations
  cumulative_distances_(0) = 0.0;
  for (Eigen::Index i = 0; i < num_segments; ++i) {
    segment_dx_(i) = path.x(i + 1) - path.x(i);
    segment_dy_(i) = path.y(i + 1) - path.y(i);
    segment_len_sq_(i) = segment_dx_(i) * segment_dx_(i) + segment_dy_(i) * segment_dy_(i);
    segment_lengths_(i) = std::sqrt(segment_len_sq_(i));
    cumulative_distances_(i + 1) = cumulative_distances_(i) + segment_lengths_(i);
  }
}
// TODO: Even I have tried other ways and spend effort maybe there can still be
// room for improvement here
float PathAlignCritic::distSqToSegment(float px, float py, Eigen::Index seg_idx)
{
  const float x1 = path_x_cache_(seg_idx);
  const float y1 = path_y_cache_(seg_idx);
  const float x2 = path_x_cache_(seg_idx + 1);
  const float y2 = path_y_cache_(seg_idx + 1);
  
  const float dx = x2 - x1;
  const float dy = y2 - y1;
  const float len_sq = dx * dx + dy * dy;
  
  if (len_sq < 1e-6f) {
    // Degenerate segment, return distance to point
    const float dpx = px - x1;
    const float dpy = py - y1;
    return dpx * dpx + dpy * dpy;
  }
  
  // Project point onto line, clamped to segment
  const float t = std::max(0.0f, std::min(1.0f, ((px - x1) * dx + (py - y1) * dy) / len_sq));
  
  const float closest_x = x1 + t * dx;
  const float closest_y = y1 + t * dy;
  
  const float dist_x = px - closest_x;
  const float dist_y = py - closest_y;
  
  return dist_x * dist_x + dist_y * dist_y;
}


float PathAlignCritic::computeMinDistanceToPath(float px, float py, Eigen::Index & closest_seg_idx)
{
  static constexpr float MIN_SEGMENT_LENGTH_SQ = 1e-6f;

  const Eigen::Index num_segments = static_cast<Eigen::Index>(path_size_cache_) - 1;
  if (num_segments < 1) {return std::numeric_limits<float>::max();}

  // Clamp hint index to valid range
  closest_seg_idx = std::clamp(closest_seg_idx, Eigen::Index(0), num_segments - 1);

  // Define search window around hint using cumulative distances along path
  const float hint_distance = cumulative_distances_(closest_seg_idx);
  const float start_distance = std::max(0.0f, hint_distance - static_cast<float>(search_window_));
  const float end_distance = hint_distance + static_cast<float>(search_window_);

  // Use binary search to find start segment for search window
  const Eigen::Index start_idx = static_cast<Eigen::Index>(
    utils::findClosestPathPtBinary(
      cumulative_distances_.data(),
      path_size_cache_,
      start_distance));

  float min_dist_sq = std::numeric_limits<float>::max();
  Eigen::Index best_seg = closest_seg_idx;

  // Search only within window for efficiency (O(window_size) instead of O(path_length))
  for (Eigen::Index seg_idx = start_idx; seg_idx < num_segments; ++seg_idx) {
    if (cumulative_distances_(seg_idx) > end_distance) {break;}
    if (segment_len_sq_(seg_idx) < MIN_SEGMENT_LENGTH_SQ) {continue;}  // Skip degenerate segments

    const float dist_sq = distSqToSegment(px, py, seg_idx);

    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      best_seg = seg_idx;
    }
  }

  // Update hint for next call
  closest_seg_idx = best_seg;
  return std::sqrt(min_dist_sq);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAlignCritic,
  mppi::critics::CriticFunction)
