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

#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

class PolygonBase
{
public:
  PolygonBase(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::string & base_frame_id,
    const double simulation_time_step);
  virtual ~PolygonBase();

  bool configure();
  void activate();
  void deactivate();

  ActionType getActionType();
  int getMaxPoints();
  double getSlowdownRatio();
  double getTimeBeforeCollision();

  virtual void getPolygon(std::vector<Point> & poly) = 0;

  // Returns estimated (simulated) time before collision
  // If there is no collision, return value will be negative.
  double getCollisionTime(
    const std::vector<Point> & points, const Velocity & velocity);

  // Returns safe velocity to keep to a collision_time before collision
  Velocity getSafeVelocity(
    const Velocity & velocity, const double collision_time);

  virtual int getPointsInside(const std::vector<Point> & points) = 0;

  void publish();

protected:
  // @brief Supporting routine obtaining all ROS-parameters
  // @param polygon_topic Output name of polygon publishing topic
  // @return Always returns true. Bool return left for unification.
  virtual bool getParameters(std::string & polygon_topic);

  // Collision Monitor node
  nav2_util::LifecycleNode::WeakPtr node_;
  // Store collision monitor node logger for further usage
  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};

  // Basic parameters
  std::string polygon_name_;
  std::string base_frame_id_;
  ActionType action_type_;
  // Maximum number of points to enter inside polygon to be ignored (without causing any action)
  int max_points_;
  // Robot slowdown (share of its actual speed)
  double slowdown_ratio_;
  // Time before collision in seconds
  double time_before_collision_;
  // Time step for robot movement simulation
  double simulation_time_step_;

  // Whether to publish this polygon
  bool visualize_;
  // Polygon points stored for later publisging
  geometry_msgs::msg::Polygon polygon_;
  // Polygon publisher for visualization purposes
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
};  // class PolygonBase

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__POLYGON_BASE_HPP_
