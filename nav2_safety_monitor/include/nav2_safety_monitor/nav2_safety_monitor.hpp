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

#ifndef NAV2_SAFETY_MONITOR__NAV2_SAFETY_MONITOR_HPP_
#define NAV2_SAFETY_MONITOR__NAV2_SAFETY_MONITOR_HPP_

#include <string>
#include <memory>
#include <std_srvs/srv/empty.hpp>
#include "nav2_safety_monitor/sensors/abstract_sensor.hpp"
#include "nav2_safety_monitor/sensors/laser.hpp"
#include "nav2_safety_monitor/sensors/sonar.hpp"
#include "nav2_safety_monitor/sensors/contact.hpp"

namespace nav2_safety_monitor
{

class SafetyMonitor
{
public:
  explicit SafetyMonitor(rclcpp::Node::SharedPtr & node);
  SafetyMonitor() = delete;
  ~SafetyMonitor();

  // getters for states of collision based on readings
  bool isInCollisionZone();
  bool isInSafetyZone();

  // process a laser scan, separated such that to run the check
  void process();

  // state info
  bool isActive();
  void activate();
  void deactivate();

protected:
  // Subscription callbacks
  void toggleCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response);

  // The ROS node to use to create publishers and subscribers
  rclcpp::Node::SharedPtr node_;

  // ROS interfaces
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr toggle_server_;

  // state and sensors
  bool active_;
  std::vector<std::unique_ptr<SafetySensor> > safety_sensors_;
};

}  // namespace nav2_safety_monitor

#endif  // NAV2_SAFETY_MONITOR__NAV2_SAFETY_MONITOR_HPP_
