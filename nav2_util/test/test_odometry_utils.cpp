// Copyright (c) 2020 Sarthak Mittal
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
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gtest/gtest.h"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

TEST(OdometryUtils, test_uninitialized)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  nav2_util::OdomSmoother odom_smoother(node, 0.3, "odom");
  geometry_msgs::msg::Twist twist_msg;
  geometry_msgs::msg::TwistStamped twist_stamped_msg;

  twist_msg = odom_smoother.getTwist();
  EXPECT_EQ(twist_msg.linear.x, 0.0);
  EXPECT_EQ(twist_msg.linear.y, 0.0);
  EXPECT_EQ(twist_msg.angular.z, 0.0);

  twist_msg = odom_smoother.getRawTwist();
  EXPECT_EQ(twist_msg.linear.x, 0.0);
  EXPECT_EQ(twist_msg.linear.y, 0.0);
  EXPECT_EQ(twist_msg.angular.z, 0.0);

  twist_stamped_msg = odom_smoother.getTwistStamped();
  EXPECT_EQ(twist_stamped_msg.twist.linear.x, 0.0);
  EXPECT_EQ(twist_stamped_msg.twist.linear.y, 0.0);
  EXPECT_EQ(twist_stamped_msg.twist.angular.z, 0.0);

  twist_stamped_msg = odom_smoother.getRawTwistStamped();
  EXPECT_EQ(twist_stamped_msg.twist.linear.x, 0.0);
  EXPECT_EQ(twist_stamped_msg.twist.linear.y, 0.0);
  EXPECT_EQ(twist_stamped_msg.twist.angular.z, 0.0);
}

TEST(OdometryUtils, test_smoothed_velocity)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

  nav2_util::OdomSmoother odom_smoother(node, 0.3, "odom");

  nav_msgs::msg::Odometry odom_msg;
  geometry_msgs::msg::Twist twist_msg;
  geometry_msgs::msg::Twist twist_raw_msg;

  auto time = node->now();

  odom_msg.header.stamp = time;
  odom_msg.twist.twist.linear.x = 1.0;
  odom_msg.twist.twist.linear.y = 1.0;
  odom_msg.twist.twist.angular.z = 1.0;

  odom_pub->publish(odom_msg);
  rclcpp::spin_some(node);

  twist_msg = odom_smoother.getTwist();
  EXPECT_EQ(twist_msg.linear.x, 1.0);
  EXPECT_EQ(twist_msg.linear.y, 1.0);
  EXPECT_EQ(twist_msg.angular.z, 1.0);

  odom_msg.header.stamp = time + rclcpp::Duration::from_seconds(0.1);
  odom_msg.twist.twist.linear.x = 2.0;
  odom_msg.twist.twist.linear.y = 2.0;
  odom_msg.twist.twist.angular.z = 2.0;
  odom_pub->publish(odom_msg);

  std::this_thread::sleep_for(100ms);
  rclcpp::spin_some(node);

  twist_msg = odom_smoother.getTwist();
  twist_raw_msg = odom_smoother.getRawTwist();
  EXPECT_EQ(twist_msg.linear.x, 1.5);
  EXPECT_EQ(twist_msg.linear.y, 1.5);
  EXPECT_EQ(twist_msg.angular.z, 1.5);
  EXPECT_EQ(twist_raw_msg.linear.x, 2.0);
  EXPECT_EQ(twist_raw_msg.linear.y, 2.0);
  EXPECT_EQ(twist_raw_msg.angular.z, 2.0);

  odom_msg.header.stamp = time + rclcpp::Duration::from_seconds(0.2);
  odom_msg.twist.twist.linear.x = 3.0;
  odom_msg.twist.twist.linear.y = 3.0;
  odom_msg.twist.twist.angular.z = 3.0;
  odom_pub->publish(odom_msg);

  std::this_thread::sleep_for(100ms);
  rclcpp::spin_some(node);

  twist_msg = odom_smoother.getTwist();
  twist_raw_msg = odom_smoother.getRawTwist();
  EXPECT_EQ(twist_msg.linear.x, 2.0);
  EXPECT_EQ(twist_msg.linear.y, 2.0);
  EXPECT_EQ(twist_msg.angular.z, 2.0);
  EXPECT_EQ(twist_raw_msg.linear.x, 3.0);
  EXPECT_EQ(twist_raw_msg.linear.y, 3.0);
  EXPECT_EQ(twist_raw_msg.angular.z, 3.0);

  odom_msg.header.stamp = time + rclcpp::Duration::from_seconds(0.45);
  odom_msg.twist.twist.linear.x = 4.0;
  odom_msg.twist.twist.linear.y = 4.0;
  odom_msg.twist.twist.angular.z = 4.0;
  odom_pub->publish(odom_msg);

  std::this_thread::sleep_for(100ms);
  rclcpp::spin_some(node);

  twist_msg = odom_smoother.getTwist();
  twist_raw_msg = odom_smoother.getRawTwist();
  EXPECT_EQ(twist_msg.linear.x, 3.5);
  EXPECT_EQ(twist_msg.linear.y, 3.5);
  EXPECT_EQ(twist_msg.angular.z, 3.5);
  EXPECT_EQ(twist_raw_msg.linear.x, 4.0);
  EXPECT_EQ(twist_raw_msg.linear.y, 4.0);
  EXPECT_EQ(twist_raw_msg.angular.z, 4.0);

  odom_msg.header.stamp = time + rclcpp::Duration::from_seconds(1.0);
  odom_msg.twist.twist.linear.x = 5.0;
  odom_msg.twist.twist.linear.y = 5.0;
  odom_msg.twist.twist.angular.z = 5.0;
  odom_pub->publish(odom_msg);

  std::this_thread::sleep_for(100ms);
  rclcpp::spin_some(node);

  twist_msg = odom_smoother.getTwist();
  twist_raw_msg = odom_smoother.getRawTwist();
  EXPECT_EQ(twist_msg.linear.x, 5.0);
  EXPECT_EQ(twist_msg.linear.y, 5.0);
  EXPECT_EQ(twist_msg.angular.z, 5.0);
  EXPECT_EQ(twist_raw_msg.linear.x, 5.0);
  EXPECT_EQ(twist_raw_msg.linear.y, 5.0);
  EXPECT_EQ(twist_raw_msg.angular.z, 5.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
