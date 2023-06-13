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
  if (fabs(vx_min) < 1e-6) {  // zero
    reversing_allowed_ = false;
  } else if (vx_min < 0.0) {   // reversing possible
    reversing_allowed_ = true;
  }

  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(offset_from_furthest_, "offset_from_furthest", 4);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 2.0);
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.5);
  getParam(
    max_angle_to_furthest_,
    "max_angle_to_furthest", 1.2);
  getParam(
    forward_preference_,
    "forward_preference", true);

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
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);
  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, data.path.x.shape(0) - 1);

  const float goal_x = xt::view(data.path.x, offseted_idx);
  const float goal_y = xt::view(data.path.y, offseted_idx);
  const float goal_yaw = xt::view(data.path.yaws, offseted_idx);
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
    goal_y - data.trajectories.y,
    goal_x - data.trajectories.x);

  auto yaws =
    xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points));

  switch (mode_) {
    case PathAngleMode::FORWARD_PREFERENCE:
      {
        data.costs += xt::pow(xt::mean(yaws, {1}, immediate) * weight_, power_);
        return;
      }
    case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
      {
        const auto yaws_between_points_corrected = xt::where(
          yaws < M_PI_2, yaws_between_points, utils::normalize_angles(yaws_between_points + M_PI));
        const auto corrected_yaws = xt::abs(
          utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points_corrected));
        data.costs += xt::pow(xt::mean(corrected_yaws, {1}, immediate) * weight_, power_);
        return;
      }
    case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
      {
        const auto yaws_between_points_corrected = xt::where(
          xt::abs(utils::shortest_angular_distance(yaws_between_points, goal_yaw)) < M_PI_2,
          yaws_between_points, utils::normalize_angles(yaws_between_points + M_PI));
        const auto corrected_yaws = xt::abs(
          utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points_corrected));
        data.costs += xt::pow(xt::mean(corrected_yaws, {1}, immediate) * weight_, power_);
        return;
      }
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAngleCritic,
  mppi::critics::CriticFunction)
