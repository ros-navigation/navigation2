// Copyright (c) 2024 Angsa Robotics
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
#include "nav2_util/odometry_utils.hpp"

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/is_stopped_condition.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class IsStoppedTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(node_);
    config_->blackboard->set(
      "odom_smoother", odom_smoother_);  // NOLINT
    // shorten duration_stopped  from default to make the test faster
    std::chrono::milliseconds duration = 100ms;
    config_->input_ports["duration_stopped"] = std::to_string(duration.count()) + "ms";
    bt_node_ = std::make_shared<nav2_behavior_tree::IsStoppedCondition>("is_stopped", *config_);
  }

  void TearDown()
  {
    bt_node_.reset();
    odom_smoother_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::IsStoppedCondition> bt_node_;
  static std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
};

std::shared_ptr<nav2_behavior_tree::IsStoppedCondition>
IsStoppedTestFixture::bt_node_ = nullptr;
std::shared_ptr<nav2_util::OdomSmoother>
IsStoppedTestFixture::odom_smoother_ = nullptr;


TEST_F(IsStoppedTestFixture, test_behavior)
{
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>(
    "odom",
    rclcpp::SystemDefaultsQoS());
  nav_msgs::msg::Odometry odom_msg;

  // Test FAILURE when robot is moving
  auto time = node_->now();
  odom_msg.header.stamp = time;
  odom_msg.twist.twist.linear.x = 1.0;
  odom_pub->publish(odom_msg);
  std::this_thread::sleep_for(10ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // Test RUNNING when robot is stopped but not long enough
  odom_msg.header.stamp = node_->now();
  odom_msg.twist.twist.linear.x = 0.001;
  odom_pub->publish(odom_msg);
  std::this_thread::sleep_for(90ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  // Test SUCCESS when robot is stopped for long enough
  std::this_thread::sleep_for(20ms);  // 20ms + 90ms = 110ms
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // Test FAILURE immediately after robot starts moving
  odom_msg.header.stamp = node_->now();
  odom_msg.twist.twist.angular.z = 0.1;
  odom_pub->publish(odom_msg);
  std::this_thread::sleep_for(10ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // Test SUCCESS when robot is stopped for long with a back-dated timestamp
  odom_msg.header.stamp = node_->now() - rclcpp::Duration(2, 0);
  odom_msg.twist.twist.angular.z = 0;
  odom_msg.twist.twist.linear.x = 0.001;
  odom_pub->publish(odom_msg);
  std::this_thread::sleep_for(10ms);  // wait just enough for the message to be received
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // Test SUCCESS when robot is stopped for long enough without a stamp for odometry
  odom_msg.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  odom_msg.twist.twist.linear.x = 0.001;
  odom_pub->publish(odom_msg);
  std::this_thread::sleep_for(10ms);
  // In the first tick, the timestamp is set
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(110ms);
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
