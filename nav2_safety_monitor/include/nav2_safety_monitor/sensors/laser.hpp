// Copyright (c) 2019 Steve Macenski
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

#ifndef NAV2_SAFETY_MONITOR__LASER_SENSOR_HPP_
#define NAV2_SAFETY_MONITOR__LASER_SENSOR_HPP_

#include <string>
#include <memory>
#include "tf2_ros/buffer.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "nav2_safety_monitor/laser_boxes.hpp"
#include "nav2_safety_monitor/sensors/abstract_sensor.hpp"

namespace nav2_safety_monitor
{

class LaserSensor : public SafetySensor
{
public:
  LaserSensor(rclcpp::Node::SharedPtr & node, std::string topic);

  virtual void process();
  virtual SafetyState getState();

  // set the safety and collision zones
  void setZones(const LaserBoxes & safety_zone, const LaserBoxes & collision_zone);

protected:
  // read and manipulate safety zones from params
  void getSafetyZone();
  void populateZoneMessages();

  bool isPtInSafetyZone(const double & x, const double & y);
  bool isPtInCollisionZone(const double & x, const double & y);
  bool IsPtInZone(const double & x, const double & y, const LaserBoxes & region);

  // Subscription callbacks
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // subscriber and publisher
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr safety_zone_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr collision_zone_pub_;
  
  // The current laser scan as received from the Laser subscription
  sensor_msgs::msg::LaserScan::SharedPtr current_scan_;

  // projector
  laser_geometry::LaserProjection projector_;
  std::unique_ptr<tf2_ros::Buffer> tf_;

  // Boxes for safety and collision zones
  LaserBoxes safety_zone_;
  LaserBoxes collision_zone_;
  geometry_msgs::msg::PolygonStamped poly_safety_, poly_collision_;

  // Timeouts after which to ignore safety state
  rclcpp::Duration safety_timeout_;
  rclcpp::Duration scan_timeout_;
  rclcpp::Time collision_start_time_;

  std::string global_frame_;
};

} // end namespace nav2_safety_monitor

#endif // NAV2_SAFETY_MONITOR__LASER_SENSOR_HPP_
