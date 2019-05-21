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

#include "nav2_safety_monitor/sensors/sonar.hpp"

namespace nav2_safety_monitor
{

SonarSensor::SonarSensor(rclcpp::Node::SharedPtr & node, std::string topic)
: SafetySensor(node, topic), safety_timeout_(0.,0.), sonar_timeout_(0.,0.)
{
  sonar_sub_ = node_->create_subscription<sensor_msgs::msg::Range>(
    topic, std::bind(&SonarSensor::sonarCallback,
    this, std::placeholders::_1));

  node_->get_parameter_or<double>("sonar_collision_range", sonar_collision_range_, 1.0);
  node_->get_parameter_or<double>("sonar_safety_range", sonar_safety_range_, 5.0);

  double safety_timeout_sec, sonar_timeout_sec;
  node_->get_parameter_or<double>("safety_timeout", safety_timeout_sec, 120.0);
  node_->get_parameter_or<double>("sonar_timeout", sonar_timeout_sec, 10.0);
  safety_timeout_ = rclcpp::Duration(safety_timeout_sec, 0.0);
  sonar_timeout_ = rclcpp::Duration(sonar_timeout_sec, 0.0);
}

void
SonarSensor::process()
{
  double range = current_measurement_->range;

  if (range < sonar_collision_range_ && range > 0.0) {
    if (current_state_ != SafetyState::COLLISION) {
      collision_start_time_ = node_->now();
      RCLCPP_INFO(node_->get_logger(),"Stopping due to imminent collision!");
    }
    current_state_ = SafetyState::COLLISION;
  } else if (range < sonar_safety_range_ && range > 0.0) {
    current_state_ = SafetyState::SLOW;
  } else {
    current_state_ = SafetyState::FREE;
  }
}

SafetyState
SonarSensor::getState()
{
  if (current_state_ == SafetyState::UNKNOWN) {
    return SafetyState::FREE;
  }

  if (node_->now() - current_measurement_->header.stamp > sonar_timeout_) {
    return SafetyState::FREE;
  }

  if (current_state_ == SafetyState::COLLISION) {
    if (node_->now() - collision_start_time_ < safety_timeout_) {
      return SafetyState::COLLISION;
    } else {
      return SafetyState::FREE;
    }
  }

  return current_state_;
}

void
SonarSensor::sonarCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  current_measurement_ = msg;
}

} // end namespace nav2_safety_monitor
