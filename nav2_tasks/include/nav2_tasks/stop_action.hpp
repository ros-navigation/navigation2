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
#include "nav2_tasks/follow_path_task.hpp"
#include "nav2_robot/robot.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_tasks
{

class StopAction : public BT::ActionNode
{
public:
  explicit StopAction(const std::string & action_name)
  : BT::ActionNode(action_name), initialized_(false)
  {
  }

  StopAction() = delete;

  ~StopAction() {}

  BT::NodeStatus tick() override
  {
    // A BT node can't get values from the blackboard in the constructor since
    // the BT library doesn't set the blackboard until after the tree if build.
    if (!initialized_) {
    node_ = blackboard()->template get<rclcpp::Node::SharedPtr>("node");

    robot_ = std::make_unique<nav2_robot::Robot>(node_);

    controller_client_ = std::make_unique<nav2_tasks::FollowPathTaskClient>(node_);

    initialized_ = true;
    }

    static auto start_time = std::chrono::system_clock::now();

    static bool new_call = true;

    if (new_call) {
      RCLCPP_INFO(node_->get_logger(), "StopAction: cancelling path following task server");
      controller_client_->cancel();

      RCLCPP_INFO(node_->get_logger(), "StopAction: publishing zero velocity command");

      // TODO(orduno) Implement a stop method on the robot
      // robot_.stop();

      geometry_msgs::msg::Twist twist;
      twist.linear.x = 0.0;
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;
      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = 0.0;
      robot_->sendVelocity(twist);

      start_time = std::chrono::system_clock::now();
      new_call = false;
      RCLCPP_INFO(node_->get_logger(), "StopAction: stabilizing robot");
    }

    // TODO(orduno) What should this node return for a long running action

    auto sleep_time = 5s;
    RCLCPP_INFO(node_->get_logger(), "StopAction: sleeping for %d seconds",
      sleep_time.count());
    std::this_thread::sleep_for(5s);
    RCLCPP_INFO(node_->get_logger(), "StopAction: finished sleeping");

    new_call = true;

    // TODO(orduno) detect if the robot is still moving or tipping over

    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
  }

private:
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<nav2_robot::Robot> robot_;

  // For publishing a zero velocity command to the robot
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  // For stopping the path following controller from sending commands to the robot
  std::unique_ptr<nav2_tasks::FollowPathTaskClient> controller_client_;

  bool initialized_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__STOP_ACTION_HPP_
