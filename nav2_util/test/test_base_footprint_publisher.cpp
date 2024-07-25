// Copyright (c) 2023 Open Navigation LLC
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

#include <string>
#include <memory>

#include "base_footprint_publisher.hpp"
#include "gtest/gtest.h"
#include "tf2/exceptions.h"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(TestBaseFootprintPublisher, TestBaseFootprintPublisher)
{
  auto node = std::make_shared<nav2_util::BaseFootprintPublisher>();
  rclcpp::spin_some(node->get_node_base_interface());

  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());
  buffer->setCreateTimerInterface(timer_interface);
  auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer, true);

  std::string base_link = "base_link";
  std::string base_footprint = "base_footprint";

  // Publish something to TF, should fail, doesn't exist
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node->now();
  transform.header.frame_id = "test1_1";
  transform.child_frame_id = "test1";
  tf_broadcaster->sendTransform(transform);
  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_THROW(
    buffer->lookupTransform(base_link, base_footprint, tf2::TimePointZero),
    tf2::TransformException);

  // This is valid, should work now and communicate the Z-removed info
  transform.header.stamp = node->now();
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 1.0;
  transform.transform.translation.z = 1.0;
  tf_broadcaster->sendTransform(transform);
  rclcpp::Rate r(1.0);
  r.sleep();
  rclcpp::spin_some(node->get_node_base_interface());
  auto t = buffer->lookupTransform(base_link, base_footprint, tf2::TimePointZero);
  EXPECT_EQ(t.transform.translation.x, 1.0);
  EXPECT_EQ(t.transform.translation.y, 1.0);
  EXPECT_EQ(t.transform.translation.z, 0.0);
}
