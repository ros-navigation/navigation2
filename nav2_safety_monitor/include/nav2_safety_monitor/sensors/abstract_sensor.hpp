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

#ifndef NAV2_SAFETY_MONITOR__ABSTRACT_SENSOR_HPP_
#define NAV2_SAFETY_MONITOR__ABSTRACT_SENSOR_HPP_

#include "rclcpp/rclcpp.hpp"

enum class SafetyState {
  UNKNOWN = 0,
  FREE = 1,
  SLOW = 2,
  COLLISION = 3
};

class SafetySensor
{

public:
  SafetySensor(rclcpp::Node::SharedPtr & node, std::string /*topic*/)
  : node_(node), current_state_(SafetyState::UNKNOWN) {};

  virtual ~SafetySensor() {return;};
  
  virtual void process() = 0;
  virtual SafetyState getState() = 0;

protected:
  // The ROS node to use to create publishers and subscribers
  rclcpp::Node::SharedPtr node_;
  
  // Current state of collision
  SafetyState current_state_;
};

#endif
