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

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void PathAlignCritic::initialize()
{
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

  RCLCPP_INFO(
    logger_,
    "ReferenceTrajectoryCritic instantiated with %d power and %f weight",
    power_, weight_);
}

void PathAlignCritic::score(CriticData & data)
{
  // Don't apply close to goal, let the goal critics take over
  if (!enabled_ || utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.goal))
  {
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

  const size_t batch_size = data.trajectories.x.shape(0);
  auto && cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});

  // Find integrated distance in the path
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

  // Finish populating the path vector
  auto & final_pose = path[path_segments_count - 1];
  final_pose.x = data.path.x(path_segments_count - 1);
  final_pose.y = data.path.y(path_segments_count - 1);
  final_pose.theta = data.path.yaws(path_segments_count - 1);

  float summed_path_dist = 0.0f, dyaw = 0.0f;
  unsigned int num_samples = 0u;
  unsigned int path_pt = 0u;
  float traj_integrated_distance = 0.0f;

  // Get strided trajectory information
  const auto T_x = xt::view(
    data.trajectories.x, xt::all(),
    xt::range(0, _, trajectory_point_step_));
  const auto T_y = xt::view(
    data.trajectories.y, xt::all(),
    xt::range(0, _, trajectory_point_step_));
  const auto T_yaw = xt::view(
    data.trajectories.yaws, xt::all(),
    xt::range(0, _, trajectory_point_step_));
  const auto traj_sampled_size = T_x.shape(1);

  for (size_t t = 0; t < batch_size; ++t) {
    summed_path_dist = 0.0f;
    num_samples = 0u;
    traj_integrated_distance = 0.0f;
    path_pt = 0u;
    float Tx_m1 = T_x(t, 0);
    float Ty_m1 = T_y(t, 0);
    for (size_t p = 1; p < traj_sampled_size; p++) {
      const float Tx = T_x(t, p);
      const float Ty = T_y(t, p);
      dx = Tx - Tx_m1;
      dy = Ty - Ty_m1;
      Tx_m1 = Tx;
      Ty_m1 = Ty;
      traj_integrated_distance += sqrtf(dx * dx + dy * dy);
      path_pt = utils::findClosestPathPt(
        path_integrated_distances, traj_integrated_distance, path_pt);

      // The nearest path point to align to needs to be not in collision, else
      // let the obstacle critic take over in this region due to dynamic obstacles
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
    if (num_samples > 0u) {
      cost[t] = summed_path_dist / static_cast<float>(num_samples);
    } else {
      cost[t] = 0.0f;
    }
  }

  if (power_ > 1u) {
    data.costs += xt::pow(std::move(cost) * weight_, power_);
  } else {
    data.costs += std::move(cost) * weight_;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAlignCritic,
  mppi::critics::CriticFunction)
