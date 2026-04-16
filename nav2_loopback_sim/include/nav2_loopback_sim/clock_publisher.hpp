// Copyright (c) 2024, Open Navigation LLC
// Copyright (c) 2026, Dexory (Tony Najjar)
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
#include <memory>

#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace nav2_loopback_sim
{

/**
 * @brief Publishes simulated clock to /clock using wall time.
 * Uses wall timers so that it works correctly even when owned by a
 * node with use_sim_time=true.  Supports speed_factor to run sim
 * time faster or slower than real time.
 */
class ClockPublisher
{
public:
  /**
   * @brief Construct a ClockPublisher.
   * @param node             Weak pointer to the owning lifecycle node
   * @param speed_factor     Sim-time speed relative to wall time
   */
  ClockPublisher(
    nav2::LifecycleNode::WeakPtr node,
    double speed_factor = 1.0);

  /**
   * @brief Start publishing /clock
   */
  void start();
  /**
   * @brief Stop publishing /clock
   */
  void stop();
  /**
   * @brief Update the simulation speed factor
   * @param speed_factor New speed multiplier (must be positive)
   */
  void setSpeedFactor(double speed_factor);

protected:
  /**
   * @brief Wall-timer callback that advances sim time and publishes /clock
   */
  void timerCallback();
  /**
   * @brief (Re)create the wall timer based on current speed_factor
   */
  void resetTimer();

  nav2::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  static constexpr double kResolution = 0.01;      // 10ms sim-time step
  static constexpr double kMinWallPeriod = 0.001;   // 1ms / 1000 Hz max
  double speed_factor_;
  rclcpp::Time sim_time_;
  std::chrono::steady_clock::time_point last_wall_time_;
};

}  // namespace nav2_loopback_sim

#endif  // NAV2_LOOPBACK_SIM__CLOCK_PUBLISHER_HPP_
