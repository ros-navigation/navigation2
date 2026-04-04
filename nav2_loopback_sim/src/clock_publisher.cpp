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
  declare_parameter("publish_period", 0.01);
  declare_parameter("speed_factor", 1.0);
  publish_period_ = get_parameter("publish_period").as_double();
  speed_factor_ = get_parameter("speed_factor").as_double();

  clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(publish_period_),
    std::bind(&ClockPublisher::timerCallback, this));

  param_handler_ = add_on_set_parameters_callback(
    std::bind(&ClockPublisher::onParameterChange, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "Sim clock publisher started (period: %.3fs, speed: %.2fx)",
    publish_period_, speed_factor_);
}

rcl_interfaces::msg::SetParametersResult ClockPublisher::onParameterChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters) {
    if (param.get_name() == "publish_period") {
      double period = param.as_double();
      if (period <= 0.0) {
        result.successful = false;
        result.reason = "publish_period must be positive";
        return result;
      }
      publish_period_ = period;
      timer_->cancel();
      timer_ = create_wall_timer(
        std::chrono::duration<double>(publish_period_),
        std::bind(&ClockPublisher::timerCallback, this));
      RCLCPP_INFO(get_logger(), "Clock publish period changed to %.3fs", publish_period_);
    } else if (param.get_name() == "speed_factor") {
      double factor = param.as_double();
      if (factor <= 0.0) {
        result.successful = false;
        result.reason = "speed_factor must be positive";
        return result;
      }
      speed_factor_ = factor;
      RCLCPP_INFO(get_logger(), "Clock speed factor changed to %.2fx", speed_factor_);
    }
  }
  return result;
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
