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

#ifndef NAV2_SAFETY_MONITOR__CONTACT_SENSOR_HPP_
#define NAV2_SAFETY_MONITOR__CONTACT_SENSOR_HPP_

#include <string>
#include <memory>
#include "std_msgs/msg/bool.hpp"
#include "nav2_safety_monitor/sensors/abstract_sensor.hpp"

namespace nav2_safety_monitor
{

class ContactSensor : public SafetySensor
{
public:
  ContactSensor(rclcpp::Node::SharedPtr & node, std::string topic);

  virtual void process();
  virtual SafetyState getState();

protected:
  // Subscription callbacks
  void bumpCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_sub_;
  std_msgs::msg::Bool::SharedPtr current_measurement_;
};

} // end namespace nav2_safety_monitor

#endif // NAV2_SAFETY_MONITOR__CONTACT_SENSOR_HPP_
