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

#ifndef NAV2_COLLISION_MONITOR__VELOCITY_POLYGON_HPP_
#define NAV2_COLLISION_MONITOR__VELOCITY_POLYGON_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Velocity polygon class.
 * This class contains all the points of the polygon and
 * the expected condition of the velocity based polygon.
 */
class VelocityPolygon
{
public:
  /**
   * @brief VelocityPolygon constructor
   * @param node Collision Monitor node pointer
   * @param polygon_name Name of main polygon
   * @param velocity_polygon_name Name of velocity polygon
   */
  VelocityPolygon(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::string & velocity_polygon_name);
  /**
   * @brief VelocityPolygon destructor
   */
  virtual ~VelocityPolygon();

  /**
   * @brief Supporting routine obtaining velocity polygon specific ROS-parameters
   * @return True if all parameters were obtained or false in failure case
   */
  bool getParameters();

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

  /// @brief Collision Monitor node
  nav2_util::LifecycleNode::WeakPtr node_;
  /// @brief Collision monitor node logger stored for further usage
  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};

  // Basic parameters
  /// @brief Points of the polygon
  std::vector<Point> poly_;
  /// @brief Name of polygon
  std::string polygon_name_;
  /// @brief velocity_polygon_name Name of velocity polygon
  std::string velocity_polygon_name_;
  /// @brief Holonomic flag (true for holonomic, false for non-holonomic)
  bool holonomic_;
  /// @brief Maximum twist linear velocity
  double linear_max_;
  /// @brief Minimum twist linear velocity
  double linear_min_;
  /// @brief End angle of velocity direction for holonomic model
  double direction_end_angle_;
  /// @brief Start angle of velocity direction for holonomic model
  double direction_start_angle_;
  /// @brief Maximum twist rotational speed
  double theta_max_;
  /// @brief Minimum twist rotational speed
  double theta_min_;
};  // class VelocityPolygon

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__VELOCITY_POLYGON_HPP_
