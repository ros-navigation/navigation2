// Copyright (c) 2024 Angsa Robotics
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

#ifndef NAV2_UTIL__BT_LOOP_RATE_HPP_
#define NAV2_UTIL__BT_LOOP_RATE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"

namespace nav2_util
{

class LoopRate
{
public:
  explicit LoopRate(const rclcpp::Duration & period, BT::Tree * tree);
  std::chrono::nanoseconds period() const;
  bool sleep();

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Duration period_;
  rclcpp::Time last_interval_;
  BT::Tree * tree_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__BT_LOOP_RATE_HPP_
