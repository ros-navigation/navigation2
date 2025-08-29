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

void PathAngleCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  getParentParam(enforce_path_inversion_, "enforce_path_inversion", false);
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
  if (!enabled_) {
    return;
  }


  if (utils::getIntegratedPathDistanceToGoal(data, enforce_path_inversion_) < threshold_to_consider_) {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);
  auto offsetted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_,
      static_cast<size_t>(data.path.x.size()) - 1);

  const float goal_x = data.path.x(offsetted_idx);
  const float goal_y = data.path.y(offsetted_idx);
  const float goal_yaw = data.path.yaws(offsetted_idx);
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

  int last_idx = data.trajectories.y.cols() - 1;
  auto diff_y = goal_y - data.trajectories.y.col(last_idx);
  auto diff_x = goal_x - data.trajectories.x.col(last_idx);
  auto yaws_between_points = diff_y.binaryExpr(
    diff_x, [&](const float & y, const float & x){return atan2f(y, x);}).eval();

  switch (mode_) {
    case PathAngleMode::FORWARD_PREFERENCE:
      {
        auto last_yaws = data.trajectories.yaws.col(last_idx);
        auto yaws = utils::shortest_angular_distance(
          last_yaws, yaws_between_points).abs();
        if (power_ > 1u) {
          data.costs += (yaws * weight_).pow(power_);
        } else {
          data.costs += yaws * weight_;
        }
        return;
      }
    case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
      {
        auto last_yaws = data.trajectories.yaws.col(last_idx);
        auto yaws_between_points_corrected = utils::normalize_yaws_between_points(last_yaws,
          yaws_between_points);
        auto corrected_yaws = utils::shortest_angular_distance(
          last_yaws, yaws_between_points_corrected).abs();
        if (power_ > 1u) {
          data.costs += (corrected_yaws * weight_).pow(power_);
        } else {
          data.costs += corrected_yaws * weight_;
        }
        return;
      }
    case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
      {
        auto last_yaws = data.trajectories.yaws.col(last_idx);
        auto yaws_between_points_corrected = utils::normalize_yaws_between_points(goal_yaw,
          yaws_between_points);
        auto corrected_yaws = utils::shortest_angular_distance(
          last_yaws, yaws_between_points_corrected).abs();
        if (power_ > 1u) {
          data.costs += (corrected_yaws * weight_).pow(power_);
        } else {
          data.costs += corrected_yaws * weight_;
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
