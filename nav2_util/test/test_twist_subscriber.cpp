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

#include "nav2_util/twist_subscriber.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
// #include "nav2_util/lifecycle_utils.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
// #include "rclcpp_lifecycle/lifecycle_publisher.hpp"

// using nav2_util::startup_lifecycle_nodes;
// using nav2_util::reset_lifecycle_nodes;

TEST(TwistPublisher, Unstamped)
{
  rclcpp::init(0, nullptr);
  auto sub_node = std::make_shared<nav2_util::LifecycleNode>("pub_node", "");
  sub_node->configure();
  sub_node->activate();

  geometry_msgs::msg::TwistStamped sub_msg {};
  auto vel_subscriber = std::make_unique<nav2_util::TwistSubscriber>(
    sub_node, "cmd_vel", 1,
    [&](const geometry_msgs::msg::Twist msg) {sub_msg.twist = msg;},
    [&](const geometry_msgs::msg::TwistStamped msg) {sub_msg = msg;}
  );
//   ASSERT_EQ(vel_publisher->get_subscription_count(), 0);
//   EXPECT_FALSE(vel_publisher->is_activated());
//   pub_node->activate();
//   EXPECT_TRUE(vel_publisher->is_activated());
//   vel_publisher->on_activate();
//   auto pub_thread = std::thread([&]() {rclcpp::spin(pub_node->get_node_base_interface());});

//   auto sub_node = std::make_shared<nav2_util::LifecycleNode>("sub_node", "");
//   pub_node->configure();


//   geometry_msgs::msg::TwistStamped pub_msg {};
//   pub_msg.twist.linear.x = 42.0;

//   geometry_msgs::msg::Twist sub_msg {};
//   auto my_sub = sub_node->create_subscription<geometry_msgs::msg::Twist>(
//     "cmd_vel", 10,
//     [&](const geometry_msgs::msg::Twist msg) {sub_msg = msg;});

//   vel_publisher->publish(pub_msg);
//   rclcpp::spin_some(sub_node->get_node_base_interface());

//   EXPECT_EQ(pub_msg.twist.linear.x, sub_msg.linear.x);
//   ASSERT_EQ(vel_publisher->get_subscription_count(), 1);
//   pub_node->deactivate();
  sub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
//   pub_thread.join();
}