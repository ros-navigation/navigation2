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

#ifndef NAV2_SAFETY_MONITOR__SONAR_SENSOR_HPP_
#define NAV2_SAFETY_MONITOR__SONAR_SENSOR_HPP_

#include <string>
#include <memory>
#include <sensor_msgs/msg/range.hpp>
#include "nav2_safety_monitor/sensors/abstract_sensor.hpp"

namespace nav2_safety_monitor
{

class SonarSensor : public SafetySensor
{
public:
  SonarSensor(rclcpp::Node::SharedPtr & node, std::string topic);

  virtual void process();
  virtual SafetyState getState();

protected:
  // Subscription callbacks
  void sonarCallback(const sensor_msgs::msg::Range::SharedPtr msg);

  // subscribers
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_sub_;
  sensor_msgs::msg::Range::SharedPtr current_measurement_;

  // params
  double sonar_collision_range_, sonar_safety_range_;

  // Timeouts after which to ignore safety state
  rclcpp::Duration safety_timeout_;
  rclcpp::Duration sonar_timeout_;
  rclcpp::Time collision_start_time_;
};

} // end namespace nav2_safety_monitor

#endif // NAV2_SAFETY_MONITOR__CONTACT_SENSOR_HPP_
