// Copyright (c) 2026
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
#include <limits>
#include <memory>
#include <thread>

#include "nav2_behavior_tree/plugins/action/get_forward_velocity_action.hpp"
#include "utils/test_behavior_tree_fixture.hpp"

using namespace std::chrono_literals;  // NOLINT

class GetForwardVelocityTest : public nav2_behavior_tree::BehaviorTreeTestFixture {};

TEST_F(GetForwardVelocityTest, fresh_and_unavailable_odometry)
{
  auto smoother = std::make_shared<nav2_util::OdomSmoother>(node_);
  config_->blackboard->set("odom_smoother", smoother);
  config_->output_ports["forward_velocity"] = "{forward_velocity}";
  config_->input_ports["max_odom_age"] = "1.0";
  nav2_behavior_tree::GetForwardVelocity action("velocity", *config_);
  EXPECT_EQ(action.executeTick(), BT::NodeStatus::FAILURE);

  auto publisher = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  publisher->on_activate();
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = node_->now();
  msg.twist.twist.linear.x = 0.42;
  for (int i = 0; i < 50; ++i) {
    publisher->publish(msg);
    std::this_thread::sleep_for(10ms);
    if (action.executeTick() == BT::NodeStatus::SUCCESS) {
      break;
    }
  }
  EXPECT_EQ(action.executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(config_->blackboard->get<double>("forward_velocity"), 0.42);

  msg.header.stamp = node_->now() - rclcpp::Duration::from_seconds(2.0);
  publisher->publish(msg);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(action.executeTick(), BT::NodeStatus::FAILURE);

  msg.header.stamp = node_->now();
  msg.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  publisher->publish(msg);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(action.executeTick(), BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
