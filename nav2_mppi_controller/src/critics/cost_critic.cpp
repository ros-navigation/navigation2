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

#include <cmath>
#include "nav2_mppi_controller/critics/cost_critic.hpp"

namespace mppi::critics
{

void CostCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(consider_footprint_, "consider_footprint", false);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.81);
  getParam(critical_cost_, "critical_cost", 300.0);
  getParam(collision_cost_, "collision_cost", 1000000.0);
  getParam(near_goal_distance_, "near_goal_distance", 0.5);
  getParam(inflation_layer_name_, "inflation_layer_name", std::string(""));

  // Normalized by cost value to put in same regime as other weights
  weight_ /= 254.0f;

  // Normalize weight when parameter is changed dynamically as well
  auto weightDynamicCb = [&](const rclcpp::Parameter & weight) {
      weight_ = weight.as_double() / 254.0f;
    };
  parameters_handler_->addDynamicParamCallback(name_ + ".cost_weight", weightDynamicCb);

  collision_checker_.setCostmap(costmap_);
  possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);

  if (possibly_inscribed_cost_ < 1.0f) {
    RCLCPP_ERROR(
      logger_,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  RCLCPP_INFO(
    logger_,
    "InflationCostCritic instantiated with %d power and %f / %f weights. "
    "Critic will collision check based on %s cost.",
    power_, critical_cost_, weight_, consider_footprint_ ?
    "footprint" : "circular");
}

float CostCritic::findCircumscribedCost(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  bool inflation_layer_found = false;
  double result = -1.0;
  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
  if (static_cast<float>(circum_radius) == circumscribed_radius_) {
    // early return if footprint size is unchanged
    return circumscribed_cost_;
  }

  // check if the costmap has an inflation layer
  for (auto layer = costmap->getLayeredCostmap()->getPlugins()->begin();
    layer != costmap->getLayeredCostmap()->getPlugins()->end();
    ++layer)
  {
    auto inflation_layer = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(*layer);
    if (!inflation_layer ||
      (!inflation_layer_name_.empty() &&
      inflation_layer->getName() != inflation_layer_name_))
    {
      continue;
    }

    inflation_layer_found = true;
    const double resolution = costmap->getCostmap()->getResolution();
    result = inflation_layer->computeCost(circum_radius / resolution);
  }

  if (!inflation_layer_found) {
    RCLCPP_WARN(
      logger_,
      "No inflation layer found in costmap configuration. "
      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
      "field to speed up collision checking by only checking the full footprint "
      "when robot is within possibly-inscribed radius of an obstacle. This may "
      "significantly slow down planning times and not avoid anything but absolute collisions!");
  }

  circumscribed_radius_ = static_cast<float>(circum_radius);
  circumscribed_cost_ = static_cast<float>(result);

  return circumscribed_cost_;
}

void CostCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;
  if (!enabled_) {
    return;
  }

  if (consider_footprint_) {
    // footprint may have changed since initialization if user has dynamic footprints
    possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);
  }

  // If near the goal, don't apply the preferential term since the goal is near obstacles
  bool near_goal = false;
  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.goal)) {
    near_goal = true;
  }

  auto && repulsive_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  repulsive_cost.fill(0.0);

  const size_t traj_len = data.trajectories.x.shape(1);
  bool all_trajectories_collide = true;
  for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
    bool trajectory_collide = false;
    const auto & traj = data.trajectories;
    float pose_cost;

    for (size_t j = 0; j < traj_len; j++) {
      // The costAtPose doesn't use orientation
      // The footprintCostAtPose will always return "INSCRIBED" if footprint is over it
      // So the center point has more information than the footprint
      pose_cost = costAtPose(traj.x(i, j), traj.y(i, j));
      if (pose_cost < 1.0f) {continue;}  // In free space

      if (inCollision(pose_cost, traj.x(i, j), traj.y(i, j), traj.yaws(i, j))) {
        trajectory_collide = true;
        break;
      }

      // Let near-collision trajectory points be punished severely
      // Note that we collision check based on the footprint actual,
      // but score based on the center-point cost regardless
      using namespace nav2_costmap_2d; // NOLINT
      if (pose_cost >= INSCRIBED_INFLATED_OBSTACLE) {
        repulsive_cost[i] += critical_cost_;
      } else if (!near_goal) {  // Generally prefer trajectories further from obstacles
        repulsive_cost[i] += pose_cost;
      }
    }

    if (!trajectory_collide) {
      all_trajectories_collide = false;
    } else {
      repulsive_cost[i] = collision_cost_;
    }
  }

  data.costs += xt::pow((weight_ * repulsive_cost / traj_len), power_);
  data.fail_flag = all_trajectories_collide;
}

/**
  * @brief Checks if cost represents a collision
  * @param cost Costmap cost
  * @return bool if in collision
  */
bool CostCritic::inCollision(float cost, float x, float y, float theta)
{
  bool is_tracking_unknown =
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

  // If consider_footprint_ check footprint scort for collision
  if (consider_footprint_ &&
    (cost >= possibly_inscribed_cost_ || possibly_inscribed_cost_ < 1.0f))
  {
    cost = static_cast<float>(collision_checker_.footprintCostAtPose(
        x, y, theta, costmap_ros_->getRobotFootprint()));
  }

  switch (static_cast<unsigned char>(cost)) {
    using namespace nav2_costmap_2d; // NOLINT
    case (LETHAL_OBSTACLE):
      return true;
    case (INSCRIBED_INFLATED_OBSTACLE):
      return consider_footprint_ ? false : true;
    case (NO_INFORMATION):
      return is_tracking_unknown ? false : true;
  }

  return false;
}

float CostCritic::costAtPose(float x, float y)
{
  using namespace nav2_costmap_2d;   // NOLINT
  unsigned int x_i, y_i;
  if (!collision_checker_.worldToMap(x, y, x_i, y_i)) {
    return nav2_costmap_2d::NO_INFORMATION;
  }

  return collision_checker_.pointCost(x_i, y_i);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::CostCritic,
  mppi::critics::CriticFunction)
