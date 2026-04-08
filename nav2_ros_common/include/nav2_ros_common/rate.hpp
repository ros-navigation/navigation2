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

#ifndef NAV2_ROS_COMMON__RATE_HPP_
#define NAV2_ROS_COMMON__RATE_HPP_

#include <memory>

#include "rclcpp/rate.hpp"
#include "rclcpp/clock.hpp"

namespace nav2
{

/**
 * @class nav2::Rate
 * @brief An rclcpp::Rate that uses the node's clock when use_sim_time is true,
 * and RCL_STEADY_TIME (monotonic, NTP-immune) otherwise.
 *
 * This is a workaround for https://github.com/ros2/rclcpp/issues/3122:
 * RCL_ROS_TIME falls back to RCL_SYSTEM_TIME when sim time is not active,
 * but rate-control loops should use RCL_STEADY_TIME on real hardware to
 * avoid timing discontinuities from NTP adjustments.
 */
class Rate : public rclcpp::Rate
{
public:
  /**
   * @brief Constructor
   * @param rate Frequency in Hz
   * @param node The node (used to query use_sim_time and get the clock)
   */
  template<typename NodeT>
  explicit Rate(const double rate, NodeT node)
  : rclcpp::Rate(
      rate,
      node->get_parameter("use_sim_time").as_bool() ?
      node->get_clock() :
      std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME))
  {}
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__RATE_HPP_
