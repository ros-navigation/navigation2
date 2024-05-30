// Copyright (C) 2023 Ryan Friedman
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

#include <memory>
#include <string>

#include "nav2_util/twist_publisher.hpp"
#include "nav2_util/lifecycle_utils.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

using nav2_util::startup_lifecycle_nodes;
using nav2_util::reset_lifecycle_nodes;

TEST(TwistPublisher, Unstamped)
{
  rclcpp::init(0, nullptr);
  auto pub_node = std::make_shared<nav2_util::LifecycleNode>("pub_node", "");
  pub_node->configure();
  auto vel_publisher = std::make_unique<nav2_util::TwistPublisher>(pub_node, "cmd_vel", 1);
  ASSERT_EQ(vel_publisher->get_subscription_count(), 0);
  EXPECT_FALSE(vel_publisher->is_activated());
  pub_node->activate();
  EXPECT_TRUE(vel_publisher->is_activated());
  vel_publisher->on_activate();
  auto pub_thread = std::thread([&]() {rclcpp::spin(pub_node->get_node_base_interface());});

  auto sub_node = std::make_shared<nav2_util::LifecycleNode>("sub_node", "");
  sub_node->configure();
  sub_node->activate();

  auto pub_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  pub_msg->twist.linear.x = 42.0;
  auto pub_msg_copy = pub_msg->twist;

  geometry_msgs::msg::Twist sub_msg {};
  auto my_sub = sub_node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    [&](const geometry_msgs::msg::Twist msg) {sub_msg = msg;});

  vel_publisher->publish(std::move(pub_msg));
  rclcpp::spin_some(sub_node->get_node_base_interface());

  EXPECT_EQ(pub_msg_copy.linear.x, sub_msg.linear.x);
  EXPECT_EQ(vel_publisher->get_subscription_count(), 1);
  pub_node->deactivate();
  sub_node->deactivate();
  rclcpp::shutdown();
  // // Have to join thread after rclcpp is shut down otherwise test hangs.
  pub_thread.join();
}

TEST(TwistPublisher, Stamped)
{
  rclcpp::init(0, nullptr);
  auto pub_node = std::make_shared<nav2_util::LifecycleNode>("pub_node", "");
  pub_node->declare_parameter("enable_stamped_cmd_vel", true);
  pub_node->configure();
  auto vel_publisher = std::make_unique<nav2_util::TwistPublisher>(pub_node, "cmd_vel", 1);
  ASSERT_EQ(vel_publisher->get_subscription_count(), 0);
  EXPECT_FALSE(vel_publisher->is_activated());
  pub_node->activate();
  EXPECT_TRUE(vel_publisher->is_activated());
  auto pub_thread = std::thread([&]() {rclcpp::spin(pub_node->get_node_base_interface());});

  auto sub_node = std::make_shared<nav2_util::LifecycleNode>("sub_node", "");
  sub_node->configure();
  sub_node->activate();

  auto pub_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  pub_msg->twist.linear.x = 42.0;
  pub_msg->header.frame_id = "foo";
  auto pub_msg_copy = *pub_msg;

  geometry_msgs::msg::TwistStamped sub_msg {};
  auto my_sub = sub_node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 10,
    [&](const geometry_msgs::msg::TwistStamped msg) {sub_msg = msg;});

  vel_publisher->publish(std::move(pub_msg));
  rclcpp::spin_some(sub_node->get_node_base_interface());
  ASSERT_EQ(vel_publisher->get_subscription_count(), 1);
  EXPECT_EQ(pub_msg_copy, sub_msg);
  pub_node->deactivate();
  sub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  pub_thread.join();
}
