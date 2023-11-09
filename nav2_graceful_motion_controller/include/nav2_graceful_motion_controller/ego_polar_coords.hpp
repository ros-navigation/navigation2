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

#ifndef NAV2_GRACEFUL_MOTION_CONTROLLER__TYPES_HPP_
#define NAV2_GRACEFUL_MOTION_CONTROLLER__TYPES_HPP_

#include <math.h>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/utils.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_graceful_motion_controller
{

/**
 * @brief Egocentric polar coordinates defined as the difference between the
 * robot pose and the target pose relative to base location and orientation.
 */
struct EgocentricPolarCoordinates
{
  double r;       // Distance between the robot pose and the target pose.
  double phi;     // Orientation of target with respect to the line of sight
                  // from the robot to the target.
  double delta;   // Steering angle of the robot with respect to the line of sight.

  EgocentricPolarCoordinates(
    const double & r_in = 0.0, const double & phi_in = 0.0,
    const double & delta_in = 0.0)
  : r(r_in), phi(phi_in), delta(delta_in) {}

  /**
   * @brief Construct a new Egocentric Polar Coordinates using the target pose.
   * 
   * @param target Target pose
   */
  explicit EgocentricPolarCoordinates(const geometry_msgs::msg::Pose & target)
  {
    r = sqrt(target.position.x * target.position.x + target.position.y * target.position.y);
    delta = std::atan2(-target.position.y, target.position.x);
    phi = angles::normalize_angle(tf2::getYaw(target.orientation) + delta);
  }

  /**
   * @brief Construct a new Egocentric Polar Coordinates using the current and target pose 
   * both in the same frame.
   * 
   * @param current Current pose
   * @param target Target pose
   */
  explicit EgocentricPolarCoordinates(
    const geometry_msgs::msg::Pose & current,
    const geometry_msgs::msg::Pose & target)
  {
    float dX = target.position.x - current.position.x;
    float dY = target.position.y - current.position.y;
    r = sqrt(dX * dX + dY * dY);
    delta = std::atan2(-dY, dX);
    phi = angles::normalize_angle(tf2::getYaw(target.orientation) + delta);
  }
};

}  // namespace nav2_graceful_motion_controller

#endif  // NAV2_GRACEFUL_MOTION_CONTROLLER__TYPES_HPP_
