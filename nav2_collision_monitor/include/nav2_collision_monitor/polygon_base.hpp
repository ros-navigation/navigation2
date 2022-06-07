// Copyright (c) 2022 Samsung Research Russia
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

#ifndef NAV2_COLLISION_MONITOR__POLYGON_BASE_HPP_
#define NAV2_COLLISION_MONITOR__POLYGON_BASE_HPP_

#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Basic polygon shape class.
 * For STOP/SLOWDOWN model it represents safety area around the robot
 * while for APPROACH model it represents robot footprint.
 */
class PolygonBase
{
public:
  /**
   * @brief PolygonBase constructor
   */
  PolygonBase(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::string & base_frame_id,
    const double simulation_time_step);
  /**
   * @brief PolygonBase destructor
   */
  virtual ~PolygonBase();

  /**
   * @brief Shape configuration routine. Obtains ROS-parameters related to shape object
   * and creates polygon lifecycle publisher.
   * @return True in case of everytging is configured correctly, or false otherwise
   */
  bool configure();
  /**
   * @brief Activates polygon lifecycle publisher
   */
  void activate();
  /**
   * @brief Deactivates polygon lifecycle publisher
   */
  void deactivate();

  /**
   * @brief Obtains shape action type
   * @return Action type for current shape
   */
  ActionType getActionType();
  /**
   * @brief Obtains shape maximum allowed points.
   * Applicable for STOP and SLOWDOWN models.
   * @return Maximum points allowed to enter to current shape
   */
  int getMaxPoints();
  /**
   * @brief Obtains speed slowdown ratio for current shape.
   * Applicable for SLOWDOWN model.
   * @return Speed slowdown ratio
   */
  double getSlowdownRatio();
  /**
   * @brief Obtains time before collision for current shape.
   * Applicable for APPROACH model.
   * @return Time before collision in seconds
   */
  double getTimeBeforeCollision();

  /**
   * @brief Gets polygon points
   * Empty virtual method intended to be used in child implementations.
   * @param poly Output polygon points (vertices)
   */
  virtual void getPolygon(std::vector<Point> & poly) = 0;

  /**
   * @brief Gets number of points inside given shape.
   * Empty virtual method intended to be used in child implementations.
   * @param points Input array of points to be checked
   * @return Number of points inside shape. If there are no points,
   * returns zero-value
   */
  virtual int getPointsInside(const std::vector<Point> & points) = 0;

  /**
   * @brief Obtains estimated (simulated) time before a collision.
   * Applicable for APPROACH model.
   * @param points Array of collision points (in a base_frame_id)
   * @param velocity Robot velocity
   * @return Estimated time before a collision. If there is no collision,
   * return value will be negative.
   */
  double getCollisionTime(
    const std::vector<Point> & points, const Velocity & velocity);

  /**
   * @brief Obtains safe velocity to keep to a constant time before a collision.
   * Applicable for APPROACH model.
   * @param velocity Robot velocity
   * @param collision_time Time before a collision (in seconds)
   * @return Safe velocity
   */
  Velocity getSafeVelocity(
    const Velocity & velocity, const double collision_time);

  /**
   * @brief Publishes polygon message into a its own topic.
   */
  void publish();

protected:
  /**
   * @brief Supporting routine obtaining all ROS-parameters
   * @param polygon_topic Output name of polygon publishing topic
   * @return Always returns true. Bool return left for unification.
   */
  virtual bool getParameters(std::string & polygon_topic);

  /// @brief Collision Monitor node
  nav2_util::LifecycleNode::WeakPtr node_;
  /// @brief Collision monitor node logger stored for further usage
  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};

  // ----- Variables -----

  // Basic parameters
  /// @brief Name of shape
  std::string polygon_name_;
  /// @brief Robot base frame ID
  std::string base_frame_id_;
  /// @brief Action type for the shape
  ActionType action_type_;
  /// @brief Maximum number of points to enter inside polygon to be ignored (w/o causing an action)
  int max_points_;
  /// @brief Robot slowdown (share of its actual speed)
  double slowdown_ratio_;
  /// @brief Time before collision in seconds
  double time_before_collision_;
  /// @brief Time step for robot movement simulation
  double simulation_time_step_;

  /// @brief Whether to publish this polygon
  bool visualize_;
  /// @brief Polygon points stored for later publisging
  geometry_msgs::msg::Polygon polygon_;
  /// @brief Polygon publisher for visualization purposes
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
};  // class PolygonBase

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__POLYGON_BASE_HPP_
