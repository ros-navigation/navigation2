// Copyright (c) 2026, Open Navigation LLC
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

#include <limits>

#include "nav2_mppi_controller/critics/corridor_critic.hpp"
#include "nav2_costmap_2d/inflation_layer_interface.hpp"

namespace mppi::critics
{

void CorridorCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1u);
  getParam(weight_, "cost_weight", 5.0f);
  getParam(robot_radius_, "robot_radius", 0.0f);
  getParam(max_corridor_width_, "max_corridor_width", 3.0f);
  getParam(threshold_to_consider_, "threshold_to_consider", 0.5f);
  getParam(inflation_layer_name_, "inflation_layer_name", std::string(""));

  // If robot_radius was not explicitly set, pull the inscribed radius from the costmap
  if (robot_radius_ <= 0.0f) {
    robot_radius_ = static_cast<float>(
      costmap_ros_->getLayeredCostmap()->getInscribedRadius());
  }

  // Resolve InflationLayer (must be the new version that exposes ESDF)
  const auto inflation_layer_if = nav2_costmap_2d::InflationLayerInterface::getInflationLayer(
    costmap_ros_, inflation_layer_name_);
  if (inflation_layer_if) {
    inflation_layer_ =
      std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(inflation_layer_if);
  }

  if (!inflation_layer_) {
    RCLCPP_ERROR(
      logger_,
      "CorridorCritic requires an InflationLayer (not LegacyInflationLayer) "
      "that exposes ESDF distances. No compatible layer found. Critic disabled.");
    enabled_ = false;
    return;
  }

  RCLCPP_INFO(
    logger_,
    "CorridorCritic instantiated with %d power, %.2f weight, "
    "robot_radius=%.3f m, max_corridor_width=%.2f m.",
    power_, weight_, robot_radius_, max_corridor_width_);
}

void CorridorCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  // Skip when very close to the goal — the path near the goal may have little
  // clearance, and other critics handle the final approach
  if (data.state.local_path_length < threshold_to_consider_) {
    return;
  }

  const Eigen::Index path_len =
    static_cast<Eigen::Index>(data.path.x.size());
  const int traj_len = static_cast<int>(data.trajectories.x.cols());
  const int batch_size = static_cast<int>(data.trajectories.x.rows());

  if (path_len < 2) {
    return;
  }

  // ------------------------------------------------------------------
  // 1. Precompute corridor widths at every path point.
  //    width[k] = clamp(esdf(path[k]) - robot_radius, 0, max_corridor_width)
  // ------------------------------------------------------------------
  const Eigen::ArrayXf corridor_widths =
    nav2_costmap_2d::SafeCorridorComputer::computeWidths(
      data.path.x.head(path_len),
      data.path.y.head(path_len),
      *inflation_layer_,
      *costmap_,
      robot_radius_,
      max_corridor_width_);

  // ------------------------------------------------------------------
  // 2. Score trajectories.
  //
  // For each time step j:
  //   a) Find the reference path point by matching the mean trajectory
  //      position at step j to the nearest path point (O(path_len)).
  //   b) Compute cross-track distance for all batch samples vectorially.
  //   c) Add max(0, dist - corridor_width) to each sample's cost.
  // ------------------------------------------------------------------
  Eigen::ArrayXf cost = Eigen::ArrayXf::Zero(batch_size);

  // ArrayXXf is column-major: column j starts at data() + j * batch_size,
  // giving a contiguous block of batch_size floats we can Map as ArrayXf.
  for (int j = 0; j < traj_len; j++) {
    Eigen::Map<const Eigen::ArrayXf> tx_j(
      data.trajectories.x.data() + j * batch_size, batch_size);
    Eigen::Map<const Eigen::ArrayXf> ty_j(
      data.trajectories.y.data() + j * batch_size, batch_size);

    // Representative position for this time step: mean over all samples
    const float mean_x = tx_j.mean();
    const float mean_y = ty_j.mean();

    // Find the nearest path point to the mean position
    Eigen::Index nk = 0;
    float min_dist2 = std::numeric_limits<float>::max();
    for (Eigen::Index k = 0; k < path_len; k++) {
      const float dx = mean_x - data.path.x(k);
      const float dy = mean_y - data.path.y(k);
      const float d2 = dx * dx + dy * dy;
      if (d2 < min_dist2) {
        min_dist2 = d2;
        nk = k;
      }
    }

    const float corridor_w = corridor_widths(nk);
    const float ref_x = data.path.x(nk);
    const float ref_y = data.path.y(nk);

    // Vectorized cross-track distances and corridor excess for all samples
    const Eigen::ArrayXf dx = tx_j - ref_x;
    const Eigen::ArrayXf dy = ty_j - ref_y;
    const Eigen::ArrayXf dist = (dx.square() + dy.square()).sqrt();

    cost += (dist - corridor_w).max(0.0f);
  }

  // Normalize by trajectory length so cost is per-step, then weight and apply power
  const float norm = weight_ / static_cast<float>(traj_len);
  if (power_ > 1u) {
    data.costs += (norm * cost).pow(static_cast<float>(power_));
  } else {
    data.costs += norm * cost;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::CorridorCritic,
  mppi::critics::CriticFunction)
