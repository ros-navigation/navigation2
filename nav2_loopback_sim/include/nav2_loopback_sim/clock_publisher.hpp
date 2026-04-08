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

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace nav2_loopback_sim
{

/**
 * @brief Publishes simulated clock to /clock using wall time.
 * Uses wall timers so that it works correctly even when owned by a
 * node with use_sim_time=true.  Supports speed_factor to run sim
 * time faster or slower than real time.
 *
 * Takes individual node interfaces rather than a concrete node type
 * so that it is decoupled from rclcpp::Node vs LifecycleNode, matching
 * the pattern used by rclcpp free functions (create_wall_timer, etc.).
 */
class ClockPublisher
{
public:
  /**
   * @brief Construct a ClockPublisher that uses the given node's interfaces.
   * @param node_base        Node base interface (for timer context)
   * @param node_timers      Node timers interface (to create wall timer)
   * @param node_topics      Node topics interface (to create publisher)
   * @param node_logging     Node logging interface (for RCLCPP_INFO / WARN)
   * @param speed_factor     Sim-time speed relative to wall time
   */
  ClockPublisher(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    double speed_factor = 1.0);

  /// Start publishing /clock
  void start();
  /// Stop publishing /clock
  void stop();

  void setSpeedFactor(double speed_factor);
  double getSpeedFactor() const {return speed_factor_;}

private:
  void timerCallback();
  void resetTimer();

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

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
