// Copyright (c) 2018 Intel Corporation
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

#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/is_stuck_condition.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class IsStuckTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    bt_node_ = std::make_shared<nav2_behavior_tree::IsStuckCondition>("is_stuck", *config_);
  }

  void TearDown()
  {
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::IsStuckCondition> bt_node_;
};

std::shared_ptr<nav2_behavior_tree::IsStuckCondition>
IsStuckTestFixture::bt_node_ = nullptr;

TEST_F(IsStuckTestFixture, test_behavior)
{
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  nav_msgs::msg::Odometry odom_msg;

  // fill up odometry history with zero velocity
  auto time = node_->now();
  odom_msg.header.stamp = time;
  odom_msg.twist.twist.linear.x = 0.0;
  odom_pub->publish(odom_msg);
  std::this_thread::sleep_for(500ms);
  odom_pub->publish(odom_msg);
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // huge negative velocity to simulate sudden brake
  odom_msg.header.stamp = time + rclcpp::Duration::from_seconds(0.1);
  odom_msg.twist.twist.linear.x = -1.5;
  odom_pub->publish(odom_msg);
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // huge positive velocity means robot is not stuck anymore
  odom_msg.header.stamp = time + rclcpp::Duration::from_seconds(0.2);
  odom_msg.twist.twist.linear.x = 1.0;
  odom_pub->publish(odom_msg);
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // stuck again due to negative velocity change is smaller time period
  odom_msg.header.stamp = time + rclcpp::Duration::from_seconds(0.25);
  odom_msg.twist.twist.linear.x = 0.0;
  odom_pub->publish(odom_msg);
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
