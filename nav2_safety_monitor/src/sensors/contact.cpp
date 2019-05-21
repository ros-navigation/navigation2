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

#include "nav2_safety_monitor/sensors/contact.hpp"

namespace nav2_safety_monitor
{

ContactSensor::ContactSensor(rclcpp::Node::SharedPtr & node, std::string topic)
: SafetySensor(node, topic)
{
  contact_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic, std::bind(&ContactSensor::bumpCallback,
    this, std::placeholders::_1));
}

void
ContactSensor::process()
{
  if (current_measurement_->data) {
    RCLCPP_INFO(node_->get_logger(),"Stopping due to imminent collision!");
    current_state_ = SafetyState::COLLISION;
  } else {
    current_state_ = SafetyState::FREE;
  }
}

SafetyState
ContactSensor::getState()
{
  if (current_state_ == SafetyState::UNKNOWN) {
    return SafetyState::FREE;
  }

  return current_state_;
}

void
ContactSensor::bumpCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  current_measurement_ = msg;
}

} // end namespace nav2_safety_monitor
