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
#include "nav2_mppi_controller/critics/obstacles_critic.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_core/controller_exceptions.hpp"

namespace mppi::critics
{

void ObstaclesCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(consider_footprint_, "consider_footprint", false);
  getParam(power_, "cost_power", 1);
  getParam(repulsion_weight_, "repulsion_weight", 1.5f);
  getParam(critical_weight_, "critical_weight", 20.0f);
  getParam(collision_cost_, "collision_cost", 100000.0f);
  getParam(collision_margin_distance_, "collision_margin_distance", 0.10f);
  getParam(near_goal_distance_, "near_goal_distance", 0.5f);
  getParam(inflation_layer_name_, "inflation_layer_name", std::string(""));
  getParam(trajectory_point_step_, "trajectory_point_step", 2);

  //collision_checker_.setCostmap(costmap_);
  collision_checker_ = std::make_unique<nav2_mppi_controller::MPPICollisionChecker>(
    costmap_ros_, parent_.lock());
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
    "in both the costmap and the obstacle critic.");
    if (costmap_ros_->getUseRadius()) {
      throw nav2_core::ControllerException(
      "Considering footprint in collision checking but no robot footprint provided in the "
      "costmap.");
    }
  }

  RCLCPP_INFO(
    logger_,
    "ObstaclesCritic instantiated with %d power and %f / %f weights. "
    "Critic will collision check based on %s cost.",
    power_, critical_weight_, repulsion_weight_, consider_footprint_ ?
    "footprint" : "circular");

  RCLCPP_WARN(logger_,
  "Inflation: radius=%.3f scale=%.3f, possible_collision_cost=%.1f, inscribed=%.3f",
  inflation_radius_, inflation_scale_factor_, possible_collision_cost_,
  costmap_ros_->getLayeredCostmap()->getInscribedRadius());

}

float ObstaclesCritic::findCircumscribedCost(
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
    result = inflation_layer->computeCost(circum_radius / resolution);
    inflation_scale_factor_ = static_cast<float>(inflation_layer->getCostScalingFactor());
    inflation_radius_ = static_cast<float>(inflation_layer->getInflationRadius());
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

float ObstaclesCritic::distanceToObstacle(const CollisionCost & cost)
{
  const float scale_factor = inflation_scale_factor_;
  const float min_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  float dist_to_obj = (scale_factor * min_radius - log(cost.cost) + log(253.0f)) / scale_factor;

  // If not footprint collision checking, the cost is using the center point cost and
  // needs the radius subtracted to obtain the closest distance to the object
  if (!cost.using_footprint) {
    dist_to_obj -= min_radius;
  }

  return dist_to_obj;
}

void ObstaclesCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  collision_checker_->setFootprint(
    costmap_ros_->getRobotFootprint(),
    costmap_ros_->getUseRadius(),
    inflation_layer_name_);

  const auto & fp = costmap_ros_->getRobotFootprint();
  RCLCPP_WARN(logger_,
    "[DBG] setFootprint: use_radius=%d, fp_pts=%zu, infl_radius=%.3f, infl_scale=%.3f, circ_cost=%.1f, inscribed=%.3f",
    costmap_ros_->getUseRadius(), fp.size(),
    inflation_radius_, inflation_scale_factor_, possible_collision_cost_,
    costmap_ros_->getLayeredCostmap()->getInscribedRadius());
  if (!fp.empty()) {
    float minx=fp[0].x, maxx=fp[0].x, miny=fp[0].y, maxy=fp[0].y;
    for (auto &p: fp) { minx=std::min(minx,(float)p.x); maxx=std::max(maxx,(float)p.x);
                        miny=std::min(miny,(float)p.y); maxy=std::max(maxy,(float)p.y); }
    RCLCPP_WARN(logger_, "[DBG] footprint bounds: x[%.3f, %.3f], y[%.3f, %.3f]", minx, maxx, miny, maxy);
  }

  if (consider_footprint_) {
    // footprint may have changed since initialization if user has dynamic footprints
    possible_collision_cost_ = findCircumscribedCost(costmap_ros_);
  }

  Eigen::ArrayXf raw_cost = Eigen::ArrayXf::Zero(data.costs.size());
  Eigen::ArrayXf repulsive_cost = Eigen::ArrayXf::Zero(data.costs.size());

  //const unsigned int traj_len = data.trajectories.x.cols();
  //const unsigned int batch_size = data.trajectories.x.rows();
  const int traj_cols_full = data.trajectories.x.cols();
  const int traj_rows = data.trajectories.x.rows();
  const int strided_cols = std::floor((traj_cols_full - 1) / trajectory_point_step_) + 1;

  bool all_trajectories_collide = true;

    // Initialize trajectories collision tracking
  // if (!data.trajectories_in_collision) {
  //   data.trajectories_in_collision = std::vector<bool>(batch_size, false);
  // }
  if (data.trajectories_in_collision && data.trajectories_in_collision->size() != (size_t)traj_rows) {
    data.trajectories_in_collision->assign(traj_rows, false);
  }



  // for(unsigned int i = 0; i != batch_size; i++) {
  for (int i = 0; i < traj_rows; ++i) {
    bool trajectory_collide = false;
    // float traj_cost = 0.0f;
    // const auto & traj = data.trajectories;
    // CollisionCost pose_cost;
    // raw_cost(i) = 0.0f;
    // repulsive_cost(i) = 0.0f;
    float& near_collision_acc = raw_cost(i);
    float& repulse_acc        = repulsive_cost(i);
    near_collision_acc = 0.0f;
    repulse_acc        = 0.0f;
    


    // for(unsigned int j = 0; j != traj_len; j++) {
    //   pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), traj.yaws(i, j));
    //   if (pose_cost.cost < 1.0f) {continue;}  // In free space

    //   if (inCollision(pose_cost.cost)) {
    //     trajectory_collide = true;
    //     break;
    //   }

    //   // Cannot process repulsion if inflation layer does not exist
    //   if (inflation_radius_ == 0.0f || inflation_scale_factor_ == 0.0f) {
    //     continue;
    //   }

    //   const float dist_to_obj = distanceToObstacle(pose_cost);

    //   // Let near-collision trajectory points be punished severely
    //   if (dist_to_obj < collision_margin_distance_) {
    //     traj_cost += (collision_margin_distance_ - dist_to_obj);
    //   }

    //   // Generally prefer trajectories further from obstacles
    //   if (!near_goal) {
    //     repulsive_cost[i] += inflation_radius_ - dist_to_obj;
    //   }
    // }

          // 1) Zbierz próbki w stride (dokładnie jak w CostCritic)
      std::vector<float> xs; xs.reserve(strided_cols);
      std::vector<float> ys; ys.reserve(strided_cols);
      std::vector<float> yaws; yaws.reserve(strided_cols);
  
      for (int j = 0; j < strided_cols; ++j) {
        const int col = j * trajectory_point_step_;
        if (col >= traj_cols_full) break;
        xs.push_back(data.trajectories.x(i, col));
        ys.push_back(data.trajectories.y(i, col));
        yaws.push_back(data.trajectories.yaws(i, col));
      }
  
      // 2) Jedno batched sprawdzenie
      auto result = collision_checker_->inCollision(
          xs, ys, yaws,
          /* treat_unknown_as_obstacle = */ false);
        
      if (i == 0) {
        auto vminmax = [](const std::vector<float>& v){
          if (v.empty()) return std::make_pair(0.f, 0.f);
          float mn=v[0], mx=v[0];
          for (auto& z: v) { mn=std::min(mn,z); mx=std::max(mx,z); }
          return std::make_pair(mn,mx);
        };
        auto cc = vminmax(result.center_cost);
        auto fc = vminmax(result.footprint_cost);
        RCLCPP_WARN(logger_,
          "[DBG] inCollision: in=%d, N=%zu, center[min=%.1f max=%.1f], footprint[min=%.1f max=%.1f]",
          (int)result.in_collision, result.center_cost.size(),
          cc.first, cc.second, fc.first, fc.second);

        // pokaż też kilka próbek z trajektorii
        for (size_t s = 0; s < std::min<size_t>(5, xs.size()); ++s) {
          RCLCPP_WARN(logger_, "[DBG] sample[%zu]: x=%.3f y=%.3f yaw=%.3f", s, xs[s], ys[s], yaws[s]);
        }
      }
  
      if (result.in_collision) {
        trajectory_collide = true;
        near_collision_acc = collision_cost_;
      } else {
        // 3) „Near-collision”    preferencja dystansu – identycznie jak u Ciebie,
        //    ale karmione danymi z 'result'
        for (size_t k = 0; k < result.center_cost.size(); ++k) {
          const bool have_fp = (k < result.footprint_cost.size() && result.footprint_cost[k] >= 0.0f);
          float cost = have_fp ? result.footprint_cost[k] : result.center_cost[k];
          if (cost < 1.0f) continue; // free
  
          float dist = distanceToObstacle(CollisionCost{cost, have_fp});
  
          // „twardy” element: kara tuż przed kolizją
          if (dist < collision_margin_distance_) {
            near_collision_acc += (collision_margin_distance_ - dist);
          }
          // „miękki” element: delikatna preferencja dalej od przeszkód
          if (inflation_radius_ > 0.f && inflation_scale_factor_ > 0.f && data.state.local_path_length >= near_goal_distance_) {
            repulse_acc += std::max(0.f, inflation_radius_ - dist);
          }
        }
      }

    // if (!trajectory_collide) {all_trajectories_collide = false;}
    // (*data.trajectories_in_collision)[i] = trajectory_collide;
    // raw_cost(i) = trajectory_collide ? collision_cost_ : traj_cost;
      if (!trajectory_collide) { all_trajectories_collide = false; }
        if (data.trajectories_in_collision) {
          (*data.trajectories_in_collision)[i] = trajectory_collide;
      }
  }

  // Normalize repulsive cost by trajectory length & lowest score to not overweight importance
  // This is a preferential cost, not collision cost, to be tuned relative to desired behaviors
  //auto repulsive_cost_normalized = (repulsive_cost - repulsive_cost.minCoeff()) / traj_len;
  auto repulsive_cost_normalized = (repulsive_cost - repulsive_cost.minCoeff()) / std::max(1, strided_cols);

  if (power_ > 1u) {
    data.costs +=
      ((critical_weight_ * raw_cost) + (repulsion_weight_ * repulsive_cost_normalized)).pow(power_);
  } else {
    data.costs += (critical_weight_ * raw_cost) + (repulsion_weight_ * repulsive_cost_normalized);
  }

  static int dbg_counter = 0;
  if (++dbg_counter % 20 == 0) {
    int coll_count = 0;
    for (int i = 0; i < traj_rows; ++i) {
      if (data.trajectories_in_collision && (*data.trajectories_in_collision)[i]) coll_count++;
    }
    RCLCPP_WARN(logger_,
      "MPPI ObstaclesCritic: rows=%d, samples/row≈%d, collide_rows=%d, raw_cost_min=%.3f max=%.3f rep_min=%.3f max=%.3f",
      traj_rows, strided_cols,
      coll_count,
      raw_cost.minCoeff(), raw_cost.maxCoeff(),
      repulsive_cost.minCoeff(), repulsive_cost.maxCoeff());
  }

  data.fail_flag = all_trajectories_collide;
}

/**
  * @brief Checks if cost represents a collision
  * @param cost Costmap cost
  * @return bool if in collision
  */
bool ObstaclesCritic::inCollision(float cost) const
{
  bool is_tracking_unknown =
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

  using namespace nav2_costmap_2d; // NOLINT
  switch (static_cast<unsigned char>(cost)) {
    case (LETHAL_OBSTACLE):
      return true;
    case (INSCRIBED_INFLATED_OBSTACLE):
      return consider_footprint_ ? false : true;
    case (NO_INFORMATION):
      return is_tracking_unknown ? false : true;
  }

  return false;
}

// CollisionCost ObstaclesCritic::costAtPose(float x, float y, float theta)
// {
//   CollisionCost collision_cost;
//   float & cost = collision_cost.cost;
//   collision_cost.using_footprint = false;
//   unsigned int x_i, y_i;
//   if (!collision_checker_.worldToMap(x, y, x_i, y_i)) {
//     cost = nav2_costmap_2d::NO_INFORMATION;
//     return collision_cost;
//   }
//   cost = collision_checker_.pointCost(x_i, y_i);

//   if (consider_footprint_ &&
//     (cost >= possible_collision_cost_ || possible_collision_cost_ < 1.0f))
//   {
//     cost = static_cast<float>(collision_checker_.footprintCostAtPose(
//         x, y, theta, costmap_ros_->getRobotFootprint()));
//     collision_cost.using_footprint = true;
//   }

//   return collision_cost;
// }

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ObstaclesCritic,
  mppi::critics::CriticFunction)
