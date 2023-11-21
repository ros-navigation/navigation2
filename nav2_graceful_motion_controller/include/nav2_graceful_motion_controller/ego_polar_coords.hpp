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

#ifndef NAV2_GRACEFUL_MOTION_CONTROLLER__EGO_POLAR_COORDS_HPP_
#define NAV2_GRACEFUL_MOTION_CONTROLLER__EGO_POLAR_COORDS_HPP_

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
 * robot pose and the target pose relative to the robot position and orientation.
 */
struct EgocentricPolarCoordinates
{
  double r;       // Distance between the robot pose and the target pose.
  double phi;     // Orientation of target with respect to the line of sight
                  // from the robot to the target.
  double delta;   // Steering angle of the robot with respect to the line of sight.

  // TODO(ajtudela): Add backwards direction
  EgocentricPolarCoordinates(
    const double & r_in = 0.0,
    const double & phi_in = 0.0,
    const double & delta_in = 0.0)
  : r(r_in), phi(phi_in), delta(delta_in) {}

  /**
   * @brief Construct a new Egocentric Polar Coordinates using the current and target pose
   * both in the same frame.
   *
   * @param target Target pose.
   * @param current Current pose. Defaults to the origin.
   */
  explicit EgocentricPolarCoordinates(
    const geometry_msgs::msg::Pose & target,
    const geometry_msgs::msg::Pose & current = geometry_msgs::msg::Pose())
  {
    float dX = target.position.x - current.position.x;
    float dY = target.position.y - current.position.y;
    float line_of_sight = std::atan2(-dY, dX);

    r = sqrt(dX * dX + dY * dY);
    phi = angles::normalize_angle(tf2::getYaw(target.orientation) + line_of_sight);
    delta = tf2::getYaw(current.orientation) + line_of_sight;
  }
};

}  // namespace nav2_graceful_motion_controller

#endif  // NAV2_GRACEFUL_MOTION_CONTROLLER__EGO_POLAR_COORDS_HPP_
