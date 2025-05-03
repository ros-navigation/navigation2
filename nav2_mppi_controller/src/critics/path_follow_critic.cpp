// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "nav2_mppi_controller/critics/path_follow_critic.hpp"

#include <Eigen/Dense>

namespace mppi::critics
{

void PathFollowCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  getParentParam(enforce_path_inversion_, "enforce_path_inversion", false);

  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 1.4f);
  getParam(offset_from_furthest_, "offset_from_furthest", 6);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0f);
}

void PathFollowCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  geometry_msgs::msg::Pose active_goal;
  if (enforce_path_inversion_) {
    active_goal = utils::getLastPathPose(data.path);
  } else {
    active_goal = data.goal;
  }

  if (data.path.x.size() < 2 ||
    utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, active_goal))
  {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);
  utils::setPathCostsIfNotSet(data, costmap_ros_);
  const size_t path_size = data.path.x.size() - 1;

  auto offsetted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, path_size);

  // Drive to the first valid path point, in case of dynamic obstacles on path
  // we want to drive past it, not through it
  bool valid = false;
  while (!valid && offsetted_idx < path_size - 1) {
    valid = (*data.path_pts_valid)[offsetted_idx];
    if (!valid) {
      offsetted_idx++;
    }
  }

  const auto path_x = data.path.x(offsetted_idx);
  const auto path_y = data.path.y(offsetted_idx);

  const int && rightmost_idx = data.trajectories.x.cols() - 1;
  const auto last_x = data.trajectories.x.col(rightmost_idx);
  const auto last_y = data.trajectories.y.col(rightmost_idx);

  const auto delta_x = last_x - path_x;
  const auto delta_y = last_y - path_y;
  if (power_ > 1u) {
    data.costs += (((delta_x.square() + delta_y.square()).sqrt()) * weight_).pow(power_);
  } else {
    data.costs += ((delta_x.square() + delta_y.square()).sqrt()) * weight_;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathFollowCritic,
  mppi::critics::CriticFunction)
