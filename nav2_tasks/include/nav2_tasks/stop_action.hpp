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

#ifndef NAV2_TASKS__STOP_ACTION_HPP_
#define NAV2_TASKS__STOP_ACTION_HPP_

#include <string>
#include <memory>
#include <cmath>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include "nav2_tasks/bt_conversions.hpp"
#include "nav2_tasks/follow_path_task.hpp"

namespace nav2_tasks
{

class StopAction : public BT::ActionNode, public rclcpp::Node
{
public:
  explicit StopAction(const std::string & action_name)
  : BT::ActionNode(action_name), Node("StopAction")
  {
    RCLCPP_INFO(get_logger(), "StopAction::constructor");

    // TODO(orduno): should we get the node from the BT::blackboard instead?
    auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](auto) {});

    controller_client_ = std::make_unique<nav2_tasks::FollowPathTaskClient>(temp_node);

    node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    // TODO(orduno) why aren't we creating the publisher like this?
    // vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  }

  StopAction() = delete;

  ~StopAction(){}

  BT::NodeStatus tick() override
  {
    using namespace std::chrono_literals;

    static auto start_time = std::chrono::system_clock::now();

    static bool new_call = true;

    if (new_call) {
      RCLCPP_INFO(get_logger(), "tick:: cancelling path following task server");
      controller_client_->cancel();

      RCLCPP_INFO(get_logger(), "tick:: publishing zero velocity command");

      // TODO(orduno) Use the robot interface instead.
      // robot_.stop();

      geometry_msgs::msg::Twist twist;
      twist.linear.x = 0.0;
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;
      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = 0.0;
      vel_pub_->publish(twist);

      start_time = std::chrono::system_clock::now();
      new_call = false;
      RCLCPP_INFO(get_logger(), "tick:: stabilizing robot");

      // return BT::NodeStatus::RUNNING;
    }

    // TODO(orduno) What should this node return for a long running action

    // Give some time to the robot to get stable
    // TODO(orduno) Use the robot interface instead
    // robot_.waitUntilStable();

    // auto current_time = std::chrono::system_clock::now();
    // std::chrono::duration<double> elapsed_time = current_time - start_time;
    // std::chrono::duration<double> hold_still_time(5);
    // std::chrono::duration<double> remaining = hold_still_time - elapsed_time;

    // if (elapsed_time < hold_still_time) {
    //   RCLCPP_INFO(get_logger(), "tick:: %.2f seconds remaining.",
    //     remaining.count());
    //   return BT::NodeStatus::RUNNING;
    // }

    // RCLCPP_INFO(get_logger(), "tick:: completed");
    // new_call = true;

    auto sleep_time = 5s;
    RCLCPP_INFO(get_logger(), "tick:: sleeping for %d seconds", sleep_time.count());
    std::this_thread::sleep_for(5s);
    RCLCPP_INFO(get_logger(), "tick:: finished sleeping");

    new_call = true;

    // TODO(orduno) detect if the robot is still moving or tipping over

    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
  }

private:
  rclcpp::Node::SharedPtr node_;

  // For publishing a zero velocity command to the robot
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  // For stopping the path following controller from sending commands to the robot
  std::unique_ptr<nav2_tasks::FollowPathTaskClient> controller_client_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__STOP_ACTION_HPP_
