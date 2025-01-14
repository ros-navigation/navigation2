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

#include "nav2_mppi_controller/critics/path_angle_critic.hpp"

#include <math.h>

namespace mppi::critics
{

using xt::evaluation_strategy::immediate;

void PathAngleCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  float vx_min;
  getParentParam(vx_min, "vx_min", -0.35);
  if (fabs(vx_min) < 1e-6f) {  // zero
    reversing_allowed_ = false;
  } else if (vx_min < 0.0f) {   // reversing possible
    reversing_allowed_ = true;
  }

  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(offset_from_furthest_, "offset_from_furthest", 4);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 2.2f);
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.5f);
  getParam(
    max_angle_to_furthest_,
    "max_angle_to_furthest", 0.785398f);

  int mode = 0;
  getParam(mode, "mode", mode);
  mode_ = static_cast<PathAngleMode>(mode);
  if (!reversing_allowed_ && mode_ == PathAngleMode::NO_DIRECTIONAL_PREFERENCE) {
    mode_ = PathAngleMode::FORWARD_PREFERENCE;
    RCLCPP_WARN(
      logger_,
      "Path angle mode set to no directional preference, but controller's settings "
      "don't allow for reversing! Setting mode to forward preference.");
  }

  RCLCPP_INFO(
    logger_,
    "PathAngleCritic instantiated with %d power and %f weight. Mode set to: %s",
    power_, weight_, modeToStr(mode_).c_str());
}

void PathAngleCritic::score(CriticData & data)
{
  if (!enabled_ ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.goal))
  {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);
  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, data.path.x.shape(0) - 1);

  const float goal_x = data.path.x(offseted_idx);
  const float goal_y = data.path.y(offseted_idx);
  const float goal_yaw = data.path.yaws(offseted_idx);
  const geometry_msgs::msg::Pose & pose = data.state.pose.pose;

  switch (mode_) {
    case PathAngleMode::FORWARD_PREFERENCE:
      if (utils::posePointAngle(pose, goal_x, goal_y, true) < max_angle_to_furthest_) {
        return;
      }
      break;
    case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
      if (utils::posePointAngle(pose, goal_x, goal_y, false) < max_angle_to_furthest_) {
        return;
      }
      break;
    case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
      if (utils::posePointAngle(pose, goal_x, goal_y, goal_yaw) < max_angle_to_furthest_) {
        return;
      }
      break;
    default:
      throw nav2_core::ControllerException("Invalid path angle mode!");
  }

  auto yaws_between_points = xt::atan2(
    goal_y - xt::view(data.trajectories.y, xt::all(), -1),
    goal_x - xt::view(data.trajectories.x, xt::all(), -1));

  switch (mode_) {
    case PathAngleMode::FORWARD_PREFERENCE:
      {
        auto yaws =
          xt::fabs(
          utils::shortest_angular_distance(
            xt::view(data.trajectories.yaws, xt::all(), -1), yaws_between_points));
        if (power_ > 1u) {
          data.costs += xt::pow(std::move(yaws) * weight_, power_);
        } else {
          data.costs += std::move(yaws) * weight_;
        }
        return;
      }
    case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
      {
        auto yaws =
          xt::fabs(
          utils::shortest_angular_distance(
            xt::view(data.trajectories.yaws, xt::all(), -1), yaws_between_points));
        const auto yaws_between_points_corrected = xt::where(
          yaws < M_PIF_2, yaws_between_points,
          utils::normalize_angles(yaws_between_points + M_PIF));
        const auto corrected_yaws = xt::fabs(
          utils::shortest_angular_distance(
            xt::view(data.trajectories.yaws, xt::all(), -1), yaws_between_points_corrected));
        if (power_ > 1u) {
          data.costs += xt::pow(std::move(corrected_yaws) * weight_, power_);
        } else {
          data.costs += std::move(corrected_yaws) * weight_;
        }
        return;
      }
    case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
      {
        const auto yaws_between_points_corrected = xt::where(
          xt::fabs(utils::shortest_angular_distance(yaws_between_points, goal_yaw)) < M_PIF_2,
          yaws_between_points, utils::normalize_angles(yaws_between_points + M_PIF));
        const auto corrected_yaws = xt::fabs(
          utils::shortest_angular_distance(
            xt::view(data.trajectories.yaws, xt::all(), -1), yaws_between_points_corrected));
        if (power_ > 1u) {
          data.costs += xt::pow(std::move(corrected_yaws) * weight_, power_);
        } else {
          data.costs += std::move(corrected_yaws) * weight_;
        }
        return;
      }
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAngleCritic,
  mppi::critics::CriticFunction)
