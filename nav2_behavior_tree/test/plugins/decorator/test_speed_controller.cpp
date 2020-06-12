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

#include "../../test_transform_handler.hpp"
#include "../../test_dummy_tree_node.hpp"
#include "nav2_behavior_tree/plugins/speed_controller.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class SpeedControllerTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    transform_handler_ = std::make_shared<nav2_behavior_tree::TransformHandler>();
    config_ = std::make_shared<BT::NodeConfiguration>();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      rclcpp::Node::SharedPtr(transform_handler_));
    config_->blackboard->set<std::shared_ptr<tf2_ros::Buffer>>(
      "tf_buffer",
      transform_handler_->getBuffer());
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(10));
    config_->blackboard->set<bool>("path_updated", false);
    config_->blackboard->set<bool>("initial_pose_received", false);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = transform_handler_->now();
    config_->blackboard->set("goal", goal);

    transform_handler_->activate();
    transform_handler_->waitForTransform();
  }

  static void TearDownTestCase()
  {
    transform_handler_->deactivate();
    delete dummy_node_;
    delete node_;
  }

  void SetUp()
  {
    node_ = new nav2_behavior_tree::SpeedController("speed_controller", *config_);
    dummy_node_ = new nav2_behavior_tree::DummyNode();
    node_->setChild(dummy_node_);
  }

  void TearDown()
  {
    dummy_node_ = nullptr;
    node_ = nullptr;
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::TransformHandler> transform_handler_;
  static std::shared_ptr<BT::NodeConfiguration> config_;
  static nav2_behavior_tree::SpeedController * node_;
  static nav2_behavior_tree::DummyNode * dummy_node_;
};

std::shared_ptr<nav2_behavior_tree::TransformHandler>
SpeedControllerTestFixture::transform_handler_ = nullptr;
std::shared_ptr<BT::NodeConfiguration> SpeedControllerTestFixture::config_ = nullptr;
nav2_behavior_tree::SpeedController * SpeedControllerTestFixture::node_ = nullptr;
nav2_behavior_tree::DummyNode * SpeedControllerTestFixture::dummy_node_ = nullptr;

/*
 * Test for speed controller behavior
 * Speed controller calculates the period after which it should succeed
 * based on the current velocity which is scaled to a pre-defined rate range
 * Current velocity is set using odom messages
 * The period is reset on the basis of current velocity after the last period is exceeded
 */
TEST_F(SpeedControllerTestFixture, test_behavior)
{
  auto odom_pub = transform_handler_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  nav_msgs::msg::Odometry odom_msg;

  auto time = transform_handler_->now();
  odom_msg.header.stamp = time;
  odom_msg.twist.twist.linear.x = 0.223;
  odom_pub->publish(odom_msg);

  EXPECT_EQ(node_->status(), BT::NodeStatus::IDLE);

  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);

  // after the first tick, period should be a default value of 1s
  // first tick should return running since period has not exceeded
  EXPECT_EQ(node_->executeTick(), BT::NodeStatus::RUNNING);

  // set the child node to success so node can return success
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);

  // should return success since period has exceeded and new period should be set to ~2s
  rclcpp::sleep_for(1s);
  EXPECT_EQ(node_->executeTick(), BT::NodeStatus::SUCCESS);

  // send new velocity for update after the next period
  odom_msg.header.stamp = time + rclcpp::Duration::from_seconds(0.5);
  odom_msg.twist.twist.linear.x = 0;
  odom_msg.twist.twist.linear.y = 0;
  odom_pub->publish(odom_msg);

  // Period should be set to ~2s based on the last speed of 0.223 m/s
  rclcpp::sleep_for(1s);
  EXPECT_EQ(node_->executeTick(), BT::NodeStatus::RUNNING);

  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  rclcpp::sleep_for(1s);
  EXPECT_EQ(node_->executeTick(), BT::NodeStatus::SUCCESS);

  // period should be set to ~10s based on the last speed of 0 m/s
  // should return running for the first 9 seconds
  for (int i = 0; i < 9; ++i) {
    rclcpp::sleep_for(1s);
    EXPECT_EQ(node_->executeTick(), BT::NodeStatus::RUNNING);
  }

  // set the child node to success so node can return success
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);

  // should return success since period has exceeded
  rclcpp::sleep_for(1s);
  EXPECT_EQ(node_->executeTick(), BT::NodeStatus::SUCCESS);
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
