// Copyright (c) 2018 Intel Corporation
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

#include <chrono>
#include <ctime>
#include <memory>

#include "nav2_recoveries/stop.hpp"

using nav2_tasks::TaskStatus;
using namespace std::chrono_literals;

namespace nav2_recoveries
{

Stop::Stop(rclcpp::Node::SharedPtr & node)
: Recovery<nav2_tasks::StopCommand, nav2_tasks::StopResult>(node)
{
  controller_client_ = std::make_unique<nav2_tasks::FollowPathTaskClient>(node_);
}

Stop::~Stop()
{
}

nav2_tasks::TaskStatus Stop::onRun(const nav2_tasks::StopCommand::SharedPtr /*command*/)
{
  // Stop the robot
  RCLCPP_INFO(node_->get_logger(), "StopAction: cancelling path following task server");
  controller_client_->cancel();

  RCLCPP_INFO(node_->get_logger(), "StopAction: publishing zero velocity command");
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
  robot_->sendVelocity(twist);

  return nav2_tasks::TaskStatus::SUCCEEDED;
}

nav2_tasks::TaskStatus Stop::onCycleUpdate(nav2_tasks::StopResult & /*result*/)
{
  // The goal is to take robot to a stopped and stable state
  // For now we only wait some time

  auto sleep_time = 5s;
  RCLCPP_INFO(node_->get_logger(), "Stop: sleeping for %d seconds",
    sleep_time.count());

  std::this_thread::sleep_for(5s);
  RCLCPP_INFO(node_->get_logger(), "Stop: finished sleeping");

  // TODO(orduno) #425 detect if the robot is still oscillating or tipping over

  return nav2_tasks::TaskStatus::SUCCEEDED;
}

}  // namespace nav2_recoveries
