// Copyright (c) 2026 Dexory (Tony Najjar)
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

#ifndef NAV2_ROS_COMMON__TIMER_HPP_
#define NAV2_ROS_COMMON__TIMER_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace nav2
{

/**
 * @brief Create a timer that uses the node's clock when use_sim_time is true,
 * and a wall timer (RCL_STEADY_TIME) otherwise.
 *
 * This is a workaround for https://github.com/ros2/rclcpp/issues/3122:
 * create_timer with the node's clock falls back to RCL_SYSTEM_TIME on real
 * hardware, but periodic timers should use RCL_STEADY_TIME to avoid timing
 * discontinuities from NTP adjustments.
 *
 * @param node The node to create the timer on
 * @param period The timer period
 * @param callback The callback to invoke
 * @param group Optional callback group
 * @return The created timer
 */
template<
  typename NodeT,
  typename DurationRepT = int64_t,
  typename DurationT = std::milli,
  typename CallbackT>
rclcpp::TimerBase::SharedPtr create_timer(
  NodeT node,
  std::chrono::duration<DurationRepT, DurationT> period,
  CallbackT && callback,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  if (node->get_parameter("use_sim_time").as_bool()) {
    return node->create_timer(period, std::forward<CallbackT>(callback), group);
  }
  return node->create_wall_timer(period, std::forward<CallbackT>(callback), group);
}

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__TIMER_HPP_
