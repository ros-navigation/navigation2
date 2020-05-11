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
#include <thread>
#include <memory>
#include <utility>

#include "dummy_controller.hpp"

using namespace std::chrono_literals;

namespace nav2_system_tests
{

DummyController::DummyController()
: Node("DummyController")
{
  RCLCPP_INFO(get_logger(), "Initializing DummyController...");

  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](auto) {});

  vel_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  task_server_ = std::make_unique<nav2_behavior_tree::FollowPathTaskServer>(temp_node, false),
  task_server_->setExecuteCallback(
    std::bind(&DummyController::followPath, this, std::placeholders::_1));

  // Start listening for incoming ComputePathToPose action server requests
  task_server_->start();

  RCLCPP_INFO(get_logger(), "Initialized DummyController");
}

DummyController::~DummyController()
{
  RCLCPP_INFO(get_logger(), "Shutting down DummyController");
}

void
DummyController::followPath(const nav2_behavior_tree::FollowPathCommand::SharedPtr /*command*/)
{
  RCLCPP_INFO(get_logger(), "Starting controller ");

  auto start_time = std::chrono::system_clock::now();
  auto time_since_msg = std::chrono::system_clock::now();

  while (true) {
    // Dummy controller computation time
    std::this_thread::sleep_for(50ms);

    if (task_server_->cancelRequested()) {
      RCLCPP_INFO(get_logger(), "Task cancelled");
      setZeroVelocity();
      task_server_->setCanceled();
      return;
    }

    // Log a message every second
    auto current_time = std::chrono::system_clock::now();
    if (current_time - time_since_msg >= 1s) {
      RCLCPP_INFO(get_logger(), "Following path");
      time_since_msg = std::chrono::system_clock::now();
    }

    // Output control command
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.x = 0.1;
    vel_pub_->publish(std::move(cmd_vel));

    if (current_time - start_time >= 30s) {
      RCLCPP_INFO(get_logger(), "Reached end point");
      setZeroVelocity();
      break;
    }
  }

  nav2_behavior_tree::FollowPathResult result;
  task_server_->setResult(result);
}

void DummyController::setZeroVelocity()
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = 0.0;
  cmd_vel->linear.y = 0.0;
  cmd_vel->angular.z = 0.0;
  vel_pub_->publish(std::move(cmd_vel));
}

}  // namespace nav2_system_tests
