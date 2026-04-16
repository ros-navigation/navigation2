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

#ifndef NAV2_BEHAVIOR_TREE__UTILS__LOOP_RATE_HPP_
#define NAV2_BEHAVIOR_TREE__UTILS__LOOP_RATE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"

namespace nav2_behavior_tree
{

class LoopRate
{
public:
  LoopRate(
    const rclcpp::Duration & period, BT::Tree * tree,
    rclcpp::Clock::SharedPtr clock)
  : clock_(clock), period_(period),
    last_interval_(clock_->now()), tree_(tree)
  {}

  // Rate-limits the BT tick loop using the provided clock (sim-time-aware).
  // Sleeps via short wall-time polls on the tree's WakeUpSignal so that
  // preemption (emitWakeUpSignal) is still handled promptly.
  bool sleep()
  {
    // Time coming into sleep
    auto now = clock_->now();
    // Time of next interval
    auto next_interval = last_interval_ + period_;
    // Detect backwards time flow
    if (now < last_interval_) {
      // Best thing to do is to set the next_interval to now + period
      next_interval = now + period_;
    }
    // Update the interval
    last_interval_ += period_;
    // If the time_to_sleep is negative or zero, don't sleep
    if (next_interval <= now) {
      // If an entire cycle was missed then reset next interval.
      // This might happen if the loop took more than a cycle.
      // Or if time jumps forward.
      if (now > next_interval + period_) {
        last_interval_ = now + period_;
      }
      // Either way do not sleep and return false
      return false;
    }
    // Poll with short wall-time waits, checking clock deadline and wake-up signal
    auto wake_up = tree_->wakeUpSignal();
    while (clock_->now() < next_interval) {
      if (wake_up && wake_up->waitFor(std::chrono::milliseconds(1))) {
        return true;
      }
    }
    return true;
  }

  std::chrono::nanoseconds period() const
  {
    return std::chrono::nanoseconds(period_.nanoseconds());
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Duration period_;
  rclcpp::Time last_interval_;
  BT::Tree * tree_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__UTILS__LOOP_RATE_HPP_
