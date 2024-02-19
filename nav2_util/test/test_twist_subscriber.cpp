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
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"


TEST(TwistSubscriber, Unstamped)
{
  rclcpp::init(0, nullptr);
  auto sub_node = std::make_shared<nav2_util::LifecycleNode>("sub_node", "");
  sub_node->configure();
  sub_node->activate();

  geometry_msgs::msg::TwistStamped sub_msg {};
  auto vel_subscriber = std::make_unique<nav2_util::TwistSubscriber>(
    sub_node, "cmd_vel", 1,
    [&](const geometry_msgs::msg::Twist msg) {sub_msg.twist = msg;},
    [&](const geometry_msgs::msg::TwistStamped msg) {sub_msg = msg;}
  );

  auto pub_node = std::make_shared<nav2_util::LifecycleNode>("pub_node", "");
  pub_node->configure();

  geometry_msgs::msg::TwistStamped pub_msg {};
  pub_msg.twist.linear.x = 42.0;

  auto vel_pub =
    pub_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  pub_node->activate();
  vel_pub->on_activate();

  vel_pub->publish(pub_msg.twist);
  rclcpp::spin_some(sub_node->get_node_base_interface());
  ASSERT_EQ(vel_pub->get_subscription_count(), 1);
  EXPECT_EQ(pub_msg, sub_msg);

  pub_node->deactivate();
  sub_node->deactivate();
  rclcpp::shutdown();
}

TEST(TwistSubscriber, Stamped)
{
  rclcpp::init(0, nullptr);
  auto sub_node = std::make_shared<nav2_util::LifecycleNode>("sub_node", "");
  sub_node->declare_parameter("enable_stamped_cmd_vel", true);
  sub_node->configure();
  sub_node->activate();

  geometry_msgs::msg::TwistStamped sub_msg {};
  auto vel_subscriber = std::make_unique<nav2_util::TwistSubscriber>(
    sub_node, "cmd_vel", 1,
    [&](const geometry_msgs::msg::Twist msg) {sub_msg.twist = msg;},
    [&](const geometry_msgs::msg::TwistStamped msg) {sub_msg = msg;}
  );

  auto pub_node = std::make_shared<nav2_util::LifecycleNode>("pub_node", "");
  pub_node->configure();

  geometry_msgs::msg::TwistStamped pub_msg {};
  pub_msg.twist.linear.x = 42.0;

  auto vel_pub =
    pub_node->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);

  pub_node->activate();
  vel_pub->on_activate();

  vel_pub->publish(pub_msg);
  rclcpp::spin_some(sub_node->get_node_base_interface());
  ASSERT_EQ(vel_pub->get_subscription_count(), 1);
  EXPECT_EQ(pub_msg, sub_msg);

  pub_node->deactivate();
  sub_node->deactivate();
  rclcpp::shutdown();
}
