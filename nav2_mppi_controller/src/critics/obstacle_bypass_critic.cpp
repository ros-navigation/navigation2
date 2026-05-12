// Copyright (c) 2026 Open Navigation LLC
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

#include <algorithm>

#include "nav2_mppi_controller/critics/obstacle_bypass_critic.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace mppi::critics
{

void ObstacleBypassCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.667f);
  getParam(min_distance_occupancy_check_, "min_distance_occupancy_check", 2.0f);
  getParam(max_path_occupancy_ratio_, "max_path_occupancy_ratio", 0.07f);
  getParam(offset_from_furthest_, "offset_from_furthest", 20);
  getParam(threshold_to_consider_, "threshold_to_consider", 0.5f);
  getParam(bypass_offset_dist_, "bypass_offset_dist", 1.0f);

  RCLCPP_INFO(
    logger_,
    "ObstacleBypassCritic instantiated with %d power and %f weight",
    power_, weight_);
}

bool ObstacleBypassCritic::determineBestBypassSide(
  float path_x, float path_y, float path_yaw, float & signed_offset)
{
  const float perp_x = -sinf(path_yaw);
  const float perp_y = cosf(path_yaw);
  const float resolution = static_cast<float>(costmap_->getResolution());
  const bool tracking_unknown = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

  const int max_steps = static_cast<int>(
    std::max(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));
  unsigned int mx, my;

  auto isNonLethal = [&](unsigned char c) {
      return c < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
        (c != nav2_costmap_2d::NO_INFORMATION || tracking_unknown);
    };

  // Scan perpendicular to the path to find the first non-lethal cell on each side.
  auto scanSide = [&](float sign) -> int {
      for (int s = 1; s <= max_steps; ++s) {
        float wx = path_x + sign * s * resolution * perp_x;
        float wy = path_y + sign * s * resolution * perp_y;
        if (!costmap_->worldToMap(wx, wy, mx, my)) {
          return max_steps + 1;
        } else if (isNonLethal(costmap_->getCost(mx, my))) {
          return s;
        }
      }
      return max_steps + 1;
    };

  const int first_free_left = scanSide(1.0f);
  const int first_free_right = scanSide(-1.0f);
  if (first_free_left > max_steps && first_free_right > max_steps) {
    return false;
  }

  // Prefer the side where free space is closer; break ties to left
  // Signed: + left, - right. Distance = first free cell + margin.
  float sign = (first_free_left <= first_free_right) ? 1.0f : -1.0f;
  int first_free = std::min(first_free_left, first_free_right);
  signed_offset = sign * (first_free * resolution + bypass_offset_dist_);

  // Validate the target point on the chosen side
  float target_x = path_x + signed_offset * perp_x;
  float target_y = path_y + signed_offset * perp_y;
  if (costmap_->worldToMap(target_x, target_y, mx, my) &&
    isNonLethal(costmap_->getCost(mx, my)))
  {
    return true;
  }

  // Try the other side
  sign = -sign;
  first_free = (sign > 0.0f) ? first_free_left : first_free_right;
  if (first_free > max_steps) {
    return false;
  }
  signed_offset = sign * (first_free * resolution + bypass_offset_dist_);
  target_x = path_x + signed_offset * perp_x;
  target_y = path_y + signed_offset * perp_y;
  if (costmap_->worldToMap(target_x, target_y, mx, my) &&
    isNonLethal(costmap_->getCost(mx, my)))
  {
    return true;
  }

  return false;
}

void ObstacleBypassCritic::score(CriticData & data)
{
  if (!enabled_ || data.state.local_path_length < threshold_to_consider_) {
    return;
  }

  // Don't apply when first getting bearing w.r.t. the path
  utils::setPathFurthestPointIfNotSet(data);
  const size_t furthest_reached_path_point = *data.furthest_reached_path_point;
  if (furthest_reached_path_point < offset_from_furthest_) {
    return;
  }

  // Find the first path IDX further than max(min_distance_occ_check, furthest_reached_path_point)
  const size_t path_segments_count = data.path.x.size() - 1;
  size_t occupancy_check_distance_idx = 0;
  float dx = 0.0f, dy = 0.0f, path_dist = 0.0f;
  for (unsigned int i = 1; i != path_segments_count; i++) {
    dx = data.path.x(i) - data.path.x(i - 1);
    dy = data.path.y(i) - data.path.y(i - 1);
    path_dist += sqrtf(dx * dx + dy * dy);
    if (path_dist <= min_distance_occupancy_check_ || i < furthest_reached_path_point) {
      occupancy_check_distance_idx = (i + 1 < path_segments_count) ? i + 1 : i;
    }
  }

  // Check if obstacles are blocking significant proportions of the local path
  // If path is blocked, incentivize turning in the shorter direction around the obstacle
  const float occupancy_check_distance_idx_flt = static_cast<float>(occupancy_check_distance_idx);
  utils::setPathCostsIfNotSet(data, costmap_ros_);
  std::vector<bool> & path_pts_valid = *data.path_pts_valid;
  float invalid_ctr = 0.0f;
  for (size_t i = 0; i < occupancy_check_distance_idx; i++) {
    if (!path_pts_valid[i]) {invalid_ctr += 1.0f;}
  }

  const float occupancy_ratio = invalid_ctr / occupancy_check_distance_idx_flt;
  const bool path_blocked = occupancy_ratio > max_path_occupancy_ratio_ && invalid_ctr > 2.0f;

  // Once bypass is active, require the ratio to drop well below the
  // threshold before deactivating to prevent oscillation
  if (!path_blocked) {
    if (!bypass_active_ || occupancy_ratio < max_path_occupancy_ratio_ * 0.5f) {
      bypass_active_ = false;
      return;
    }
  }

  // Find the first blocked path point
  size_t blocked_idx = 0;
  for (size_t j = 0; j < occupancy_check_distance_idx; j++) {
    if (!path_pts_valid[j]) {blocked_idx = j; break;}
  }

  // Find first valid path point past the blocked region
  size_t resume_idx = blocked_idx;
  for (; resume_idx < path_pts_valid.size(); resume_idx++) {
    if (path_pts_valid[resume_idx]) {break;}
  }

  // If blocked until the end of the path, don't activate bypass
  if (resume_idx >= path_pts_valid.size()) {
    bypass_active_ = false;
    return;
  }

  // Midpoint of blocked region to score against. The path is being continuously
  // pruned, so the blocked_idx is updated and adjusted forward as the robot moves
  const size_t obstacle_idx = (blocked_idx + resume_idx) / 2;

  // Compute path tangent from XY poses at the obstacle region to determine which
  // direction to steer the vehicle to attempt to bypass the obstacle.
  const size_t next_idx = std::min(obstacle_idx + 1, path_segments_count - 1);
  const float path_x = data.path.x(obstacle_idx);
  const float path_y = data.path.y(obstacle_idx);
  const float tangent_x = data.path.x(next_idx) - path_x;
  const float tangent_y = data.path.y(next_idx) - path_y;
  const float tangent_len = sqrtf(tangent_x * tangent_x + tangent_y * tangent_y);
  if (tangent_len < 1e-6f) {
    bypass_active_ = false;
    return;
  }
  const float path_yaw = atan2f(tangent_y, tangent_x);

  float signed_offset = 0.0f;
  if (!determineBestBypassSide(path_x, path_y, path_yaw, signed_offset)) {
    bypass_active_ = false;
    return;  // No valid bypass found
  }

  // Score against a forward-looking target point offset from the path
  // in the direction of the bypass to incentivize trajectories to steer around
  // the obstacle in the direction with the least disruption to path tracking.
  const size_t target_idx = std::min(
    furthest_reached_path_point + offset_from_furthest_, path_segments_count - 1);
  const size_t target_next = std::min(target_idx + 1, path_segments_count - 1);
  const float target_tx = data.path.x(target_next) - data.path.x(target_idx);
  const float target_ty = data.path.y(target_next) - data.path.y(target_idx);
  const float target_tlen = sqrtf(target_tx * target_tx + target_ty * target_ty);
  if (target_tlen < 1e-6f) {
    bypass_active_ = false;
    return;
  }
  const float perp_x = -target_ty / target_tlen;
  const float perp_y = target_tx / target_tlen;
  const float target_x = data.path.x(target_idx) + signed_offset * perp_x;
  const float target_y = data.path.y(target_idx) + signed_offset * perp_y;
  const int last_idx = data.trajectories.y.cols() - 1;
  const auto diff_x = target_x - data.trajectories.x.col(last_idx);
  const auto diff_y = target_y - data.trajectories.y.col(last_idx);

  if (power_ > 1u) {
    data.costs +=
      (((diff_x.square() + diff_y.square()).sqrt()) * weight_).pow(power_);
  } else {
    data.costs += ((diff_x.square() + diff_y.square()).sqrt()) * weight_;
  }

  bypass_active_ = true;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ObstacleBypassCritic,
  mppi::critics::CriticFunction)
