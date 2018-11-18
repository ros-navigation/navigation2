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
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include "nav2_tasks/bt_conversions.hpp"

namespace nav2_tasks
{

class StopAction : public BT::ActionNode, public rclcpp::Node
{
public:
  explicit StopAction(const std::string & action_name)
  : BT::ActionNode(action_name), Node("StopAction")
  {
    std::cout << "StopAction::StopAction" << std::endl;

    node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  }

  StopAction() = delete;

  ~StopAction(){}

  BT::NodeStatus tick() override
  {
    std::cout << "StopAction::tick: publishing stop command" << std::endl;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    vel_pub_->publish(twist);

    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__STOP_ACTION_HPP_
