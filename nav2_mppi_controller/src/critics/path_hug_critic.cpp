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
#include "nav2_util/execution_timer.hpp"

namespace mppi::critics
{

void PathHugCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0f);
  getParam(max_path_occupancy_ratio_, "max_path_occupancy_ratio", 0.07f);
  getParam(offset_from_furthest_, "offset_from_furthest", 20);
  getParam(trajectory_point_step_, "trajectory_point_step", 4);
  getParam(threshold_to_consider_, "threshold_to_consider", 0.5f);
  getParam(search_window_, "search_window", 0.15f);
  getParam(lookahead_distance_, "lookahead_distance", 0.3f);

  RCLCPP_INFO(
    logger_,
    "PathHugCritic-V2 instantiated with %d power, %f weight, search_window: %f m, lookahead: %f m",
    power_, weight_, search_window_, lookahead_distance_);
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

void PathHugCritic::score(CriticData & data)
{
  nav2_util::ExecutionTimer timer;
  timer.start();
  if (!enabled_ || data.state.local_path_length < threshold_to_consider_) {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);
  const size_t path_segments_count = *data.furthest_reached_path_point;

  utils::setPathCostsIfNotSet(data, costmap_ros_);
  std::vector<bool> & path_pts_valid = *data.path_pts_valid;

  const auto & traj_x = data.trajectories.x;
  const auto & traj_y = data.trajectories.y;
  const Eigen::Index batch_size = traj_x.rows();
  const Eigen::Index traj_length = traj_x.cols();

  const size_t num_segments = path_segments_count - 1;

  updateCumulativeDistances(data.path, num_segments);

  // Find robot's current closest segment as starting hint
  const float robot_x = data.state.pose.pose.position.x;
  const float robot_y = data.state.pose.pose.position.y;

  Eigen::Index initial_path_hint = 0;
  int dummy_segments = 0;
  computeMinDistanceToPath(robot_x, robot_y, data.path, num_segments,
                          initial_path_hint, dummy_segments);

  Eigen::ArrayXf cost_array(batch_size);
  cost_array.setZero();

  Eigen::ArrayXi valid_sample_count(batch_size);
  valid_sample_count.setZero();

  const int effective_stride = std::max(1, std::min(trajectory_point_step_,
     static_cast<int>(traj_length)));

  for (Eigen::Index traj_idx = 0; traj_idx < batch_size; ++traj_idx) {
    Eigen::Index path_hint = initial_path_hint;

    float accumulated_distance = 0.0f;
    float prev_x = traj_x(traj_idx, 0);
    float prev_y = traj_y(traj_idx, 0);
    int sample_idx = 0;

    while (true) {
      const Eigen::Index traj_col = sample_idx * effective_stride;
      if (traj_col >= traj_length) {break;}

      const float px = traj_x(traj_idx, traj_col);
      const float py = traj_y(traj_idx, traj_col);

      if (sample_idx > 0) {
        const float dx = px - prev_x;
        const float dy = py - prev_y;

        const float abs_dx = std::abs(dx);
        const float abs_dy = std::abs(dy);
        const float max_delta = std::max(abs_dx, abs_dy);
        const float min_delta = std::min(abs_dx, abs_dy);
        const float distance_traveled = max_delta * 0.96f + min_delta * 0.398f;
        accumulated_distance += distance_traveled;

        if (accumulated_distance > lookahead_distance_) {break;}
      }

      int segments_searched = 0;
      const float dist_sq = computeMinDistanceToPath(
        px, py, data.path, num_segments, path_hint, segments_searched);

      if (path_hint < static_cast<Eigen::Index>(num_segments) && path_pts_valid[path_hint]) {
        cost_array(traj_idx) += dist_sq;
        valid_sample_count(traj_idx)++;
      }

      prev_x = px;
      prev_y = py;
      sample_idx++;
    }
  }

  for (Eigen::Index i = 0; i < batch_size; ++i) {
    if (valid_sample_count(i) > 0) {
      cost_array(i) = std::sqrt(cost_array(i) / static_cast<float>(valid_sample_count(i)));
    }
  }

  if (power_ > 1u) {
    data.costs += (cost_array * weight_).pow(power_);
  } else {
    data.costs += cost_array * weight_;
  }
  timer.end();
  RCLCPP_INFO(logger_, "PathHugCritic execution time: %.3f ms",
    timer.elapsed_time_in_seconds() * 1000.0);
}

float PathHugCritic::computeMinDistanceToPath(
  float px, float py,
  const models::Path & path,
  size_t num_segments,
  Eigen::Index & path_hint,
  int & segments_searched)
{
  segments_searched = 0;

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
  segments_searched++;

  const float hint_dist = cumulative_distances_[path_hint];
  const float min_dist_threshold = hint_dist - search_window_;
  const float max_dist_threshold = hint_dist + search_window_;

  // Exponential search BACKWARD
  size_t start = static_cast<size_t>(path_hint);
  for (int step = 1; start > 0; step *= 2) {
    const size_t test = start > static_cast<size_t>(step) ? start - step : 0;
    if (cumulative_distances_[test] < min_dist_threshold) {
      break;
    }
    start = test;
    if (start == 0) {break;}
  }

  // Exponential search FORWARD
  size_t end = start;
  for (int step = 1; end < num_segments; step *= 2) {
    const size_t test = std::min(end + step, num_segments - 1);
    if (cumulative_distances_[test] > max_dist_threshold) {
      break;
    }
    end = test;
    if (end == num_segments - 1) {break;}
  }

  // Linear search in window
  for (size_t i = start; i <= end; ++i) {
    if (static_cast<Eigen::Index>(i) == path_hint) {continue;}

    const float dist_sq = distSqToSegment(i);
    segments_searched++;

    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      best_seg = static_cast<Eigen::Index>(i);
    }
  }

  path_hint = best_seg;
  return min_dist_sq;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathHugCritic,
  mppi::critics::CriticFunction)
