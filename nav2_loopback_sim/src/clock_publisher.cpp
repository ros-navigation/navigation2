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

#include "nav2_loopback_sim/clock_publisher.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace nav2_loopback_sim
{

ClockPublisher::ClockPublisher(const rclcpp::NodeOptions & options)
: Node("sim_clock_publisher", options),
  sim_time_(0, 0, RCL_ROS_TIME),
  last_wall_time_(std::chrono::steady_clock::now())
{
  declare_parameter("speed_factor", 1.0);
  speed_factor_ = get_parameter("speed_factor").as_double();

  clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  resetTimer();

  param_handler_ = add_on_set_parameters_callback(
    std::bind(&ClockPublisher::onParameterChange, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "Sim clock publisher started (resolution: %.3fs, speed: %.2fx)",
    kResolution, speed_factor_);
}

rcl_interfaces::msg::SetParametersResult ClockPublisher::onParameterChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters) {
    if (param.get_name() == "speed_factor") {
      double factor = param.as_double();
      if (factor <= 0.0) {
        result.successful = false;
        result.reason = "speed_factor must be positive";
        return result;
      }
      speed_factor_ = factor;
      resetTimer();
      RCLCPP_INFO(get_logger(), "Clock speed factor changed to %.2fx", speed_factor_);
    }
  }
  return result;
}

void ClockPublisher::resetTimer()
{
  timer_.reset();
  double wall_period = std::max(kResolution / speed_factor_, kMinWallPeriod);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(wall_period),
    std::bind(&ClockPublisher::timerCallback, this));
  if (kResolution / speed_factor_ < kMinWallPeriod) {
    RCLCPP_WARN(
      get_logger(),
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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_loopback_sim::ClockPublisher)
