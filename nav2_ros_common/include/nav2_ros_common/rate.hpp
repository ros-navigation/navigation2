// Copyright (c) 2026 Dexory
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

#ifndef NAV2_ROS_COMMON__RATE_HPP_
#define NAV2_ROS_COMMON__RATE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <utility>

#include "rclcpp/rate.hpp"
#include "rclcpp/create_timer.hpp"

namespace nav2
{

/**
 * @brief Select the appropriate clock for rate-limiting or timers.
 *
 * When use_sim_time is true, returns the node's ROS clock (simulation time).
 * When use_sim_time is false, returns a steady (monotonic) clock to avoid
 * issues with system clock jumps (e.g. NTP corrections).
 *
 * @param node Any ROS node pointer or shared_ptr
 * @return The appropriate clock
 */
template<typename NodeT>
rclcpp::Clock::SharedPtr selectSteadyOrSimClock(NodeT node)
{
  bool use_sim_time = false;
  auto params = node->get_node_parameters_interface();
  if (params->has_parameter("use_sim_time")) {
    use_sim_time = params->get_parameter("use_sim_time").as_bool();
  }
  if (use_sim_time) {
    return node->get_clock();
  }
  return std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
}

/**
 * @brief A sim-time-aware rate for Nav2 loops.
 *
 * When use_sim_time is true, the rate is governed by the node's ROS clock
 * (simulation time). When use_sim_time is false, a steady (monotonic) clock
 * is used to avoid issues with system clock jumps (e.g. NTP corrections).
 */
class Rate : public rclcpp::Rate
{
public:
  /**
   * @brief Construct a Rate from a node and frequency.
   * @param node Any ROS node pointer or shared_ptr (used to check use_sim_time and obtain clock)
   * @param rate Desired frequency in Hz
   */
  template<typename NodeT>
  explicit Rate(NodeT node, double rate)
  : rclcpp::Rate(rate, selectSteadyOrSimClock(node))
  {}
};

/**
 * @brief A steady-clock Nav2 wall rate.
 *
 * This rate always uses RCL_STEADY_TIME and is never sim-time-aware.
 */
class WallRate : public rclcpp::WallRate
{
public:
  explicit WallRate(double rate)
  : rclcpp::WallRate(rate)
  {}
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__RATE_HPP_
