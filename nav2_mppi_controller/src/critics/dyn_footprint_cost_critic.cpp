// Copyright (c) 2026
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

#include "nav2_mppi_controller/critics/dyn_footprint_cost_critic.hpp"

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/inflation_layer_interface.hpp"

namespace mppi::critics
{

void DynFootprintCostCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
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
  auto weightDynamicCb = [&](
    const rclcpp::Parameter & weight) {
      weight_ = weight.as_double() / 254.0f;
    };
  parameters_handler_->addParamCallback(name_ + ".cost_weight", weightDynamicCb);

  collision_checker_.setCostmap(costmap_);

  if (costmap_ros_->getUseRadius()) {
    throw nav2_core::ControllerException(
      "DynFootprintCostCritic requires an explicit robot footprint polygon to scale with "
      "velocity, but the costmap is configured with 'robot_radius' (circular) instead of "
      "'footprint'. Set 'footprint: \"[[x1,y1],...]\"' on the local costmap, or remove "
      "DynFootprintCostCritic from the critics list if you want to keep the circular model.");
  }

  const auto inflation_layer = nav2_costmap_2d::InflationLayerInterface::getInflationLayer(
    costmap_ros_, inflation_layer_name_);
  if (inflation_layer == nullptr) {
    throw nav2_core::ControllerException(
      "DynFootprintCostCritic requires an inflation layer on the local costmap to compute "
      "velocity-scaled footprint thresholds, but none was found (checked layer name: '" +
      inflation_layer_name_ + "'). Add an inflation_layer plugin to the local costmap, or "
      "set 'inflation_layer_name' if it is configured under a different name.");
  }
  const double resolution = costmap_ros_->getCostmap()->getResolution();

  velocity_polygon_ = std::make_shared<mppi::polygon_utils::VelocityPolygon>();
  velocity_polygon_->onConfigure(name_ + ".polygon_description", parameters_handler_);
  velocity_polygon_->computeBucketThresholds(inflation_layer, resolution);

  if (near_collision_cost_ > 253) {
    RCLCPP_WARN(logger_, "Near collision cost is set higher than INSCRIBED_INFLATED_OBSTACLE");
  }

  RCLCPP_INFO(
    logger_,
    "DynFootprintCostCritic instantiated with %d power and %f / %f weights. "
    "Critic will collision check based on a velocity-scaled footprint.",
    power_, critical_cost_, weight_);
}

void DynFootprintCostCritic::score(CriticData & data)
{
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

  // If near the goal, don't apply the preferential term since the goal is near obstacles
  bool near_goal = false;
  if (data.state.local_path_length < near_goal_distance_) {
    near_goal = true;
  }

  Eigen::ArrayXf repulsive_cost(data.costs.rows());
  repulsive_cost.setZero();
  bool all_trajectories_collide = true;

  auto & collisions = data.trajectories_in_collision;
  const bool track_collisions = !collisions.empty();

  int strided_traj_cols = floor((data.trajectories.x.cols() - 1) / trajectory_point_step_) + 1;
  int strided_traj_rows = data.trajectories.x.rows();
  int outer_stride = strided_traj_rows * trajectory_point_step_;

  const auto traj_x = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(
    data.trajectories.x.data(), strided_traj_rows, strided_traj_cols,
    Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto traj_y = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(
    data.trajectories.y.data(), strided_traj_rows, strided_traj_cols,
    Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto traj_yaw = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(
    data.trajectories.yaws.data(), strided_traj_rows, strided_traj_cols,
    Eigen::Stride<-1, -1>(outer_stride, 1));

  const auto traj_vx = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(
    data.state.vx.data(), strided_traj_rows, strided_traj_cols,
    Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto traj_vy = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(
    data.state.vy.data(), strided_traj_rows, strided_traj_cols,
    Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto traj_wz = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(
    data.state.wz.data(), strided_traj_rows, strided_traj_cols,
    Eigen::Stride<-1, -1>(outer_stride, 1));

  for (int i = 0; i < strided_traj_rows; ++i) {
    bool trajectory_collide = false;
    float pose_cost = 0.0f;
    float & traj_cost = repulsive_cost(i);

    for (int j = 0; j < strided_traj_cols; j++) {
      float Tx = traj_x(i, j);
      float Ty = traj_y(i, j);
      float Tyaw = traj_yaw(i, j);
      float Tvx = traj_vx(i, j);
      float Tvy = traj_vy(i, j);
      float Twz = traj_wz(i, j);
      unsigned int x_i = 0u, y_i = 0u;

      // The getCost doesn't use orientation
      // The footprintCostAtPose will always return "INSCRIBED" if footprint is over it
      // So the center point has more information than the footprint
      if (!worldToMapFloat(Tx, Ty, x_i, y_i)) {
        pose_cost = 255.0f;  // NO_INFORMATION in float
      } else {
        pose_cost = static_cast<float>(costmap->getCost(getIndex(x_i, y_i)));
        // removed the pose_cost <1.0 check
        // later we can compare this againts the biggest's tau_circ and can skip
        // the further computation.
      }

      // Handle the center-pose NO_INFORMATION case explicitly, before the
      // numeric tau_circumscribed/tau_inscribed comparisons
      if (static_cast<unsigned char>(pose_cost) == nav2_costmap_2d::NO_INFORMATION) {
        if (!is_tracking_unknown_) {
          traj_cost = collision_cost_;
          trajectory_collide = true;
          if (track_collisions) {collisions[i] = true;}
          break;
        }
        if (pose_cost >= static_cast<float>(near_collision_cost_)) {
          traj_cost += critical_cost_;
        } else if (!near_goal) {
          traj_cost += pose_cost;
        }
        continue;
      }


      const auto * subpoly = velocity_polygon_->findPolygon(Tvx, Tvy, Twz);

      // No configured bucket covers this sampled velocity (e.g. a gap in
      // polygon_description, or a sample beyond the configured range),
      // fall back to the costmap's nominal footprint rather than guessing.
      const nav2_costmap_2d::Footprint & exact_footprint =
        (subpoly != nullptr) ? subpoly->poly : costmap_ros_->getRobotFootprint();
      const double tau_circumscribed = (subpoly != nullptr) ? subpoly->tau_circumscribed : -1.0;
      const double tau_inscribed = (subpoly != nullptr) ? subpoly->tau_inscribed : -1.0;

      if (tau_circumscribed != -1.0 && pose_cost < tau_circumscribed) {
        // Provably safe: even the worst-case orientation of this bucket's
        // footprint cannot be touching anything.
        if (pose_cost >= static_cast<float>(near_collision_cost_)) {
          traj_cost += critical_cost_;
        } else if (!near_goal) {
          traj_cost += pose_cost;
        }
        continue;
      }

      if (tau_inscribed != -1.0 && pose_cost > tau_inscribed) {
        // Guaranteed collision, any orientation, a lethal cell is within
        // this bucket's inscribed radius of the center.
        traj_cost = collision_cost_;
        trajectory_collide = true;
        if (track_collisions) {collisions[i] = true;}
        break;
      }

      // Neither shortcut applies: orientation actually matters here, so
      // fall through to the exact per-vertex check.
      const float exact_cost = static_cast<float>(collision_checker_.footprintCostAtPose(
          Tx, Ty, Tyaw, exact_footprint));

      bool is_collision = false;
      switch (static_cast<unsigned char>(exact_cost)) {
        case nav2_costmap_2d::LETHAL_OBSTACLE:
          is_collision = true;
          break;
        case nav2_costmap_2d::NO_INFORMATION:
          is_collision = !is_tracking_unknown_;
          break;
        default:
          // Includes INSCRIBED_INFLATED_OBSTACLE and any decayed value —
          // not a confirmed footprint collision.
          is_collision = false;
          break;
      }

      if (is_collision) {
        traj_cost = collision_cost_;
        trajectory_collide = true;
        if (track_collisions) {collisions[i] = true;}
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

    all_trajectories_collide &= trajectory_collide;
  }

  if (power_ > 1u) {
    data.costs += (repulsive_cost *
      (weight_ / static_cast<float>(strided_traj_cols))).pow(power_);
  } else {
    data.costs += repulsive_cost * (weight_ / static_cast<float>(strided_traj_cols));
  }

  data.fail_flag = all_trajectories_collide;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::DynFootprintCostCritic,
  mppi::critics::CriticFunction)
