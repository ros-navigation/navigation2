// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "nav2_mppi_controller/critics/esdf_critic.hpp"
#include "nav2_costmap_2d/inflation_layer_interface.hpp"
#include "nav2_core/controller_exceptions.hpp"

namespace mppi::critics
{

void EsdfCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(consider_footprint_, "consider_footprint", false);
  getParam(power_, "cost_power", 1);
  getParam(repulsion_weight_, "repulsion_weight", 1.5f);
  getParam(critical_weight_, "critical_weight", 20.0f);
  getParam(collision_cost_, "collision_cost", 100000.0f);
  getParam(collision_margin_distance_, "collision_margin_distance", 0.10f);
  getParam(near_goal_distance_, "near_goal_distance", 0.5f);
  getParam(inflation_layer_name_, "inflation_layer_name", std::string(""));

  collision_checker_.setCostmap(costmap_);

  // Resolve the inflation layer and verify it exposes ESDF (new InflationLayer, not Legacy)
  const auto inflation_layer_if = nav2_costmap_2d::InflationLayerInterface::getInflationLayer(
    costmap_ros_, inflation_layer_name_);
  if (inflation_layer_if) {
    inflation_layer_ =
      std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(inflation_layer_if);
  }

  if (!inflation_layer_) {
    RCLCPP_ERROR(
      logger_,
      "EsdfCritic requires an InflationLayer (not LegacyInflationLayer) in the costmap. "
      "No compatible inflation layer found. The critic is disabled. "
      "Use ObstaclesCritic or CostCritic as a fallback.");
    enabled_ = false;
    return;
  }

  if (costmap_ros_->getUseRadius() == consider_footprint_) {
    RCLCPP_WARN(
      logger_,
      "Inconsistent configuration: consider_footprint=%s but the costmap uses a %s. "
      "Please verify robot shape settings.",
      consider_footprint_ ? "true" : "false",
      costmap_ros_->getUseRadius() ? "radius" : "footprint polygon");
    if (costmap_ros_->getUseRadius()) {
      throw nav2_core::ControllerException(
              "EsdfCritic: consider_footprint=true but the costmap uses a circular radius. "
              "Disable consider_footprint or provide a full footprint polygon.");
    }
  }

  inscribed_radius_ = static_cast<float>(
    costmap_ros_->getLayeredCostmap()->getInscribedRadius());
  circumscribed_radius_ = static_cast<float>(
    costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
  inflation_radius_ = static_cast<float>(inflation_layer_->getInflationRadius());

  RCLCPP_INFO(
    logger_,
    "EsdfCritic instantiated with %d power and %.2f / %.2f weights. "
    "Inflation radius: %.3f m. Collision check: %s.",
    power_, critical_weight_, repulsion_weight_, inflation_radius_,
    consider_footprint_ ? "footprint" : "circular");
}

bool EsdfCritic::inCollision(float dist, float x, float y, float theta)
{
  // On or within inscribed radius of a lethal obstacle: always a collision
  if (dist <= inscribed_radius_) {
    return true;
  }

  // Between inscribed and circumscribed: full footprint check needed
  if (consider_footprint_ && dist < circumscribed_radius_) {
    const float fp_cost = static_cast<float>(
      collision_checker_.footprintCostAtPose(
        x, y, theta, costmap_ros_->getRobotFootprint()));
    switch (static_cast<unsigned char>(fp_cost)) {
      case nav2_costmap_2d::LETHAL_OBSTACLE:
        return true;
      case nav2_costmap_2d::NO_INFORMATION:
        return !is_tracking_unknown_;
      default:
        break;
    }
  }

  return false;
}

void EsdfCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  is_tracking_unknown_ = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

  // Refresh radii in case footprint changed since initialization
  inscribed_radius_ = static_cast<float>(
    costmap_ros_->getLayeredCostmap()->getInscribedRadius());
  inflation_radius_ = static_cast<float>(inflation_layer_->getInflationRadius());

  // Cache costmap geometry for fast world→cell conversion
  auto * cm = costmap_;
  origin_x_ = static_cast<float>(cm->getOriginX());
  origin_y_ = static_cast<float>(cm->getOriginY());
  resolution_ = static_cast<float>(cm->getResolution());
  size_x_ = cm->getSizeInCellsX();
  size_y_ = cm->getSizeInCellsY();

  const bool near_goal = (data.state.local_path_length < near_goal_distance_);

  const unsigned int traj_len = data.trajectories.x.cols();
  const unsigned int batch_size = data.trajectories.x.rows();

  Eigen::ArrayXf raw_cost = Eigen::ArrayXf::Zero(data.costs.size());
  Eigen::ArrayXf repulsive_cost = Eigen::ArrayXf::Zero(data.costs.size());
  bool all_trajectories_collide = true;

  auto & collisions = data.trajectories_in_collision;
  const bool track_collisions = !collisions.empty();

  for (unsigned int i = 0; i != batch_size; i++) {
    bool trajectory_collide = false;
    float traj_critical_cost = 0.0f;
    const auto & traj = data.trajectories;

    for (unsigned int j = 0; j != traj_len; j++) {
      const float tx = traj.x(i, j);
      const float ty = traj.y(i, j);

      unsigned int mx, my;
      if (!worldToMapFloat(tx, ty, mx, my)) {
        trajectory_collide = true;
        break;
      }

      // Check for NO_INFORMATION before querying ESDF
      // (the ESDF only tracks lethal obstacles unless inflate_around_unknown is set)
      const unsigned char cell_cost = cm->getCost(mx + my * size_x_);
      if (cell_cost == nav2_costmap_2d::NO_INFORMATION) {
        if (!is_tracking_unknown_) {
          trajectory_collide = true;
          break;
        }
        continue;
      }

      const float dist = inflation_layer_->getDistanceToObstacle(mx, my);

      if (inCollision(dist, tx, ty, traj.yaws(i, j))) {
        trajectory_collide = true;
        break;
      }

      // Near-collision penalty: punish poses that are too close to obstacles
      if (dist < collision_margin_distance_) {
        traj_critical_cost += collision_margin_distance_ - dist;
      }

      // Repulsion: prefer trajectories further from obstacles
      if (!near_goal && dist < inflation_radius_) {
        repulsive_cost[i] += inflation_radius_ - dist;
      }
    }

    if (!trajectory_collide) {all_trajectories_collide = false;}
    raw_cost(i) = trajectory_collide ? collision_cost_ : traj_critical_cost;
    if (trajectory_collide && track_collisions) {collisions[i] = true;}
  }

  // Normalize repulsive cost by trajectory length so it doesn't outweigh critical cost
  const auto repulsive_cost_normalized =
    (repulsive_cost - repulsive_cost.minCoeff()) / static_cast<float>(traj_len);

  if (power_ > 1u) {
    data.costs +=
      ((critical_weight_ * raw_cost) +
      (repulsion_weight_ * repulsive_cost_normalized)).pow(power_);
  } else {
    data.costs +=
      (critical_weight_ * raw_cost) +
      (repulsion_weight_ * repulsive_cost_normalized);
  }

  data.fail_flag = all_trajectories_collide;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::EsdfCritic,
  mppi::critics::CriticFunction)
