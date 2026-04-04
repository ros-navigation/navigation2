// Copyright (c) 2026 Tony Najjar
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

#ifndef NAV2_LOOPBACK_SIM__CLOCK_PUBLISHER_HPP_
#define NAV2_LOOPBACK_SIM__CLOCK_PUBLISHER_HPP_

#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace nav2_loopback_sim
{

/**
 * @brief Publishes simulated clock to /clock using wall time.
 * Must NOT use use_sim_time, since it is the clock source.
 * Supports speed_factor to run sim time faster or slower than real time.
 */
class ClockPublisher : public rclcpp::Node
{
public:
  explicit ClockPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timerCallback();
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handler_;

  double publish_period_;
  double speed_factor_;
  rclcpp::Time sim_time_;
  std::chrono::steady_clock::time_point last_wall_time_;
};

}  // namespace nav2_loopback_sim

#endif  // NAV2_LOOPBACK_SIM__CLOCK_PUBLISHER_HPP_
