// Copyright (c) 2022 Dexory
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

#ifndef NAV2_COLLISION_MONITOR__POLYGON_VELOCITY_HPP_
#define NAV2_COLLISION_MONITOR__POLYGON_VELOCITY_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Polygon velocity class.
 * This class contains all the points of the polygon and
 * the expected condition of the velocity based polygon.
 */
class PolygonVelocity
{
public:
  /**
   * @brief PolygonVelocity constructor
   * @param poly Points of the polygon
   * @param polygon_name Name of polygon
   * @param linear_max Maximum twist linear velocity
   * @param linear_min Minimum twist linear velocity
   * @param direction_max Maximum twist direction angle
   * @param direction_min Minimum twist direction angle
   * @param theta_max Maximum twist rotational speed
   * @param theta_min Minimum twist rotational speed
   */
  PolygonVelocity(
    const std::vector<Point> & poly,
    const std::string & polygon_name,
    const double & linear_max,
    const double & linear_min,
    const double & direction_max,
    const double & direction_min,
    const double & theta_max,
    const double & theta_min);
  /**
   * @brief PolygonVelocity destructor
   */
  virtual ~PolygonVelocity();

  /**
   * @brief Check if the velocities and direction is in expected range.
   * @param cmd_vel_in Robot twist command input
   * @return True if speed and direction is within the condition
   */
  bool isInRange(const Velocity & cmd_vel_in);

  /**
   * @brief Check if the velocities and direction is in expected range.
   * @param cmd_vel_in Robot twist command input
   * @return True if speed and direction is within the condition
   */
  std::vector<Point> getPolygon();

protected:
  // ----- Variables -----

  /// @brief Collision monitor node logger stored for further usage
  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};

  // Basic parameters
  /// @brief Points of the polygon
  std::vector<Point> poly_;
  /// @brief Name of polygon
  std::string polygon_name_;
  /// @brief Maximum twist linear velocity
  double linear_max_;
  /// @brief Minimum twist linear velocity
  double linear_min_;
  /// @brief Maximum twist direction angle
  double direction_max_;
  /// @brief Minimum twist direction angle
  double direction_min_;
  /// @brief Maximum twist rotational speed
  double theta_max_;
  /// @brief Minimum twist rotational speed
  double theta_min_;
};  // class PolygonVelocity

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__POLYGON_VELOCITY_HPP_
