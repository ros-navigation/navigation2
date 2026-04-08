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

#include "nav2_loopback_sim/clock_publisher.hpp"

#include <chrono>

namespace nav2_loopback_sim
{

ClockPublisher::ClockPublisher(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  double speed_factor)
: node_base_(node_base),
  node_timers_(node_timers),
  node_logging_(node_logging),
  speed_factor_(speed_factor),
  sim_time_(0, 0, RCL_ROS_TIME),
  last_wall_time_(std::chrono::steady_clock::now())
{
  clock_pub_ = rclcpp::create_publisher<rosgraph_msgs::msg::Clock>(
    node_topics, "/clock", 10);
}

void ClockPublisher::start()
{
  last_wall_time_ = std::chrono::steady_clock::now();
  resetTimer();
  RCLCPP_INFO(
    node_logging_->get_logger(),
    "Sim clock publisher started (resolution: %.3fs, speed: %.2fx)",
    kResolution, speed_factor_);
}

void ClockPublisher::stop()
{
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
}

void ClockPublisher::setSpeedFactor(double speed_factor)
{
  if (speed_factor <= 0.0) {
    RCLCPP_WARN(
      node_logging_->get_logger(),
      "Ignoring non-positive speed_factor %.2f", speed_factor);
    return;
  }
  speed_factor_ = speed_factor;
  if (timer_) {
    resetTimer();
  }
  RCLCPP_INFO(
    node_logging_->get_logger(),
    "Clock speed factor changed to %.2fx", speed_factor_);
}

void ClockPublisher::resetTimer()
{
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  double wall_period = std::max(kResolution / speed_factor_, kMinWallPeriod);
  timer_ = rclcpp::create_wall_timer(
    std::chrono::duration<double>(wall_period),
    std::bind(&ClockPublisher::timerCallback, this),
    nullptr,
    node_base_.get(),
    node_timers_.get());
  if (kResolution / speed_factor_ < kMinWallPeriod) {
    RCLCPP_WARN(
      node_logging_->get_logger(),
      "Wall period clamped to %.1fms (requested %.3fms from resolution=%.3f, speed=%.1f)",
      kMinWallPeriod * 1000.0, (kResolution / speed_factor_) * 1000.0,
      kResolution, speed_factor_);
  }
}

void ClockPublisher::timerCallback()
{
  auto now_wall = std::chrono::steady_clock::now();
  double wall_dt = std::chrono::duration<double>(now_wall - last_wall_time_).count();
  last_wall_time_ = now_wall;

  sim_time_ += rclcpp::Duration::from_seconds(wall_dt * speed_factor_);

  rosgraph_msgs::msg::Clock msg;
  msg.clock = sim_time_;
  clock_pub_->publish(msg);
}

}  // namespace nav2_loopback_sim
