// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#ifndef NAV2_GRACEFUL_CONTROLLER__UTILS_HPP_
#define NAV2_GRACEFUL_CONTROLLER__UTILS_HPP_

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_graceful_controller
{
/**
   * @brief Create a PointStamped message of the motion target for
   * debugging / visualization porpuses.
   *
   * @param motion_target Motion target in PoseStamped format
   * @return geometry_msgs::msg::PointStamped Motion target in PointStamped format
   */
geometry_msgs::msg::PointStamped createMotionTargetMsg(
  const geometry_msgs::msg::PoseStamped & motion_target);

/**
   * @brief Create a flat circle marker of radius slowdown_radius around the motion target for
   * debugging / visualization porpuses.
   *
   * @param motion_target Motion target
   * @param slowdown_radius Radius of the slowdown circle
   * @return visualization_msgs::msg::Marker Slowdown marker
   */
visualization_msgs::msg::Marker createSlowdownMarker(
  const geometry_msgs::msg::PoseStamped & motion_target, const double & slowdown_radius);

}  // namespace nav2_graceful_controller

#endif  // NAV2_GRACEFUL_CONTROLLER__UTILS_HPP_
