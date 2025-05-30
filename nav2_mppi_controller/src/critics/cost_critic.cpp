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
#include "nav2_core/controller_exceptions.hpp"

namespace mppi::critics
{

void CostCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(consider_footprint_, "consider_footprint", false);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.81f);
  getParam(critical_cost_, "critical_cost", 300.0f);
  getParam(near_collision_cost_, "near_collision_cost", 253);
  getParam(collision_cost_, "collision_cost", 1000000.0f);
  getParam(near_goal_distance_, "near_goal_distance", 0.5f);
  getParam(inflation_layer_name_, "inflation_layer_name", std::string(""));
  getParam(trajectory_point_step_, "trajectory_point_step", 2);

  // Normalized by cost value to put in same regime as other weights
  weight_ /= 254.0f;

  // Normalize weight when parameter is changed dynamically as well
  auto weightDynamicCb = [&](const rclcpp::Parameter & weight) {
      weight_ = weight.as_double() / 254.0f;
    };
  parameters_handler_->addDynamicParamCallback(name_ + ".cost_weight", weightDynamicCb);

  collision_checker_.setCostmap(costmap_);
  possible_collision_cost_ = findCircumscribedCost(costmap_ros_);

  if (possible_collision_cost_ < 1.0f) {
    RCLCPP_ERROR(
      logger_,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  if (costmap_ros_->getUseRadius() == consider_footprint_) {
    RCLCPP_WARN(
    logger_,
    "Inconsistent configuration in collision checking. Please verify the robot's shape settings "
    "in both the costmap and the cost critic.");
    if (costmap_ros_->getUseRadius()) {
      throw nav2_core::ControllerException(
      "Considering footprint in collision checking but no robot footprint provided in the "
      "costmap.");
    }
  }

  if(near_collision_cost_ > 253) {
    RCLCPP_WARN(logger_, "Near collision cost is set higher than INSCRIBED_INFLATED_OBSTACLE");
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
  double result = -1.0;
  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
  if (static_cast<float>(circum_radius) == circumscribed_radius_) {
    // early return if footprint size is unchanged
    return circumscribed_cost_;
  }

  // check if the costmap has an inflation layer
  const auto inflation_layer = nav2_costmap_2d::InflationLayer::getInflationLayer(
    costmap,
    inflation_layer_name_);
  if (inflation_layer != nullptr) {
    const double resolution = costmap->getCostmap()->getResolution();
    double inflation_radius = inflation_layer->getInflationRadius();
    if (inflation_radius < circum_radius) {
      RCLCPP_ERROR(
        rclcpp::get_logger("computeCircumscribedCost"),
        "The inflation radius (%f) is smaller than the circumscribed radius (%f) "
        "If this is an SE2-collision checking plugin, it cannot use costmap potential "
        "field to speed up collision checking by only checking the full footprint "
        "when robot is within possibly-inscribed radius of an obstacle. This may "
        "significantly slow down planning times!",
        inflation_radius, circum_radius);
      result = 0.0;
      return result;
    }
    result = inflation_layer->computeCost(circum_radius / resolution);
  } else {
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
  using xt::placeholders::_;
  if (!enabled_) {
    return;
  }

  // Setup cost information for various parts of the critic
  is_tracking_unknown_ = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
  auto * costmap = collision_checker_.getCostmap();
  origin_x_ = static_cast<float>(costmap->getOriginX());
  origin_y_ = static_cast<float>(costmap->getOriginY());
  resolution_ = static_cast<float>(costmap->getResolution());
  size_x_ = costmap->getSizeInCellsX();
  size_y_ = costmap->getSizeInCellsY();

  if (consider_footprint_) {
    // footprint may have changed since initialization if user has dynamic footprints
    possible_collision_cost_ = findCircumscribedCost(costmap_ros_);
  }

  // If near the goal, don't apply the preferential term since the goal is near obstacles
  bool near_goal = false;
  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.goal)) {
    near_goal = true;
  }

  auto && repulsive_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  bool all_trajectories_collide = true;

  const size_t traj_len = floor(data.trajectories.x.shape(1) / trajectory_point_step_);
  const auto traj_x =
    xt::view(data.trajectories.x, xt::all(), xt::range(0, _, trajectory_point_step_));
  const auto traj_y =
    xt::view(data.trajectories.y, xt::all(), xt::range(0, _, trajectory_point_step_));
  const auto traj_yaw = xt::view(
    data.trajectories.yaws, xt::all(), xt::range(0, _, trajectory_point_step_));

  for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
    bool trajectory_collide = false;
    float pose_cost = 0.0f;
    float & traj_cost = repulsive_cost[i];
    traj_cost = 0.0f;

    for (size_t j = 0; j < traj_len; j++) {
      float Tx = traj_x(i, j);
      float Ty = traj_y(i, j);
      unsigned int x_i = 0u, y_i = 0u;

      // The getCost doesn't use orientation
      // The footprintCostAtPose will always return "INSCRIBED" if footprint is over it
      // So the center point has more information than the footprint
      if (!worldToMapFloat(Tx, Ty, x_i, y_i)) {
        pose_cost = 255.0f;  // NO_INFORMATION in float
      } else {
        pose_cost = static_cast<float>(costmap->getCost(getIndex(x_i, y_i)));
        if (pose_cost < 1.0f) {
          continue;  // In free space
        }
      }

      if (inCollision(pose_cost, Tx, Ty, traj_yaw(i, j))) {
        traj_cost = collision_cost_;
        trajectory_collide = true;
        break;
      }

      // Let near-collision trajectory points be punished severely
      // Note that we collision check based on the footprint actual,
      // but score based on the center-point cost regardless
      if (pose_cost >= static_cast<float>(near_collision_cost_)) {
        traj_cost += critical_cost_;
      } else if (!near_goal) {  // Generally prefer trajectories further from obstacles
        traj_cost += pose_cost;
      }
    }

    if (!trajectory_collide) {
      all_trajectories_collide = false;
    }
  }

  if (power_ > 1u) {
    data.costs += xt::pow(
      (std::move(repulsive_cost) * (weight_ / static_cast<float>(traj_len))), power_);
  } else {
    data.costs += std::move(repulsive_cost) * (weight_ / static_cast<float>(traj_len));
  }

  data.fail_flag = all_trajectories_collide;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::CostCritic,
  mppi::critics::CriticFunction)
