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

#ifndef NAV2_GRACEFUL_CONTROLLER__EGO_POLAR_COORDS_HPP_
#define NAV2_GRACEFUL_CONTROLLER__EGO_POLAR_COORDS_HPP_

#include <math.h>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/utils.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_graceful_controller
{

/**
 * @brief Egocentric polar coordinates defined as the difference between the
 * robot pose and the target pose relative to the robot position and orientation.
 */
struct EgocentricPolarCoordinates
{
  float r;       // Radial distance between the robot pose and the target pose.
                 // Negative value if the robot is moving backwards.
  float phi;     // Orientation of target with respect to the line of sight
                 // from the robot to the target.
  float delta;   // Steering angle of the robot with respect to the line of sight.

  EgocentricPolarCoordinates(
    const float & r_in = 0.0,
    const float & phi_in = 0.0,
    const float & delta_in = 0.0)
  : r(r_in), phi(phi_in), delta(delta_in) {}

  /**
   * @brief Construct a new egocentric polar coordinates as the difference between the robot pose
   * and the target pose relative to the robot position and orientation, both referenced to the same frame.
   *
   * Thus, r, phi and delta are always at the origin of the frame.
   *
   * @param target Target pose.
   * @param current Current pose. Defaults to the origin.
   * @param backward If true, the robot is moving backwards. Defaults to false.
   */
  explicit EgocentricPolarCoordinates(
    const geometry_msgs::msg::Pose & target,
    const geometry_msgs::msg::Pose & current = geometry_msgs::msg::Pose(), bool backward = false)
  {
    // Compute the difference between the target and the current pose
    float dX = target.position.x - current.position.x;
    float dY = target.position.y - current.position.y;
    // Compute the line of sight from the robot to the target
    // Flip it if the robot is moving backwards
    float line_of_sight = backward ? (std::atan2(-dY, dX) + M_PI) : std::atan2(-dY, dX);
    // Compute the ego polar coordinates
    r = sqrt(dX * dX + dY * dY);
    phi = angles::normalize_angle(tf2::getYaw(target.orientation) + line_of_sight);
    delta = angles::normalize_angle(tf2::getYaw(current.orientation) + line_of_sight);
    // If the robot is moving backwards, flip the sign of the radial distance
    r *= backward ? -1.0 : 1.0;
  }

  /**
   * @brief Construct a new egocentric polar coordinates for the target pose.
   *
   * @param target Target pose.
   * @param backward If true, the robot is moving backwards. Defaults to false.
   */
  explicit EgocentricPolarCoordinates(
    const geometry_msgs::msg::Pose & target,
    bool backward = false)
  {
    EgocentricPolarCoordinates(target, geometry_msgs::msg::Pose(), backward);
  }
};

}  // namespace nav2_graceful_controller

#endif  // NAV2_GRACEFUL_CONTROLLER__EGO_POLAR_COORDS_HPP_
