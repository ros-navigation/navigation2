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
#include <cmath>
#include <memory>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"

#include "../../transform_handler.hpp"
#include "../../dummy_tree_node.hpp"
#include "../../../plugins/decorator/distance_controller.hpp"

class DistanceControllerTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    transform_handler_ = new nav2_behavior_tree::TransformHandler();
    config_ = new BT::NodeConfiguration();
    dummy_node_ = new nav2_behavior_tree::DummyNode();

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

    transform_handler_->activate();
    transform_handler_->waitForTransform();
  }

  static void TearDownTestCase()
  {
    transform_handler_->deactivate();
    delete transform_handler_;
    delete config_;
    delete dummy_node_;
    transform_handler_ = nullptr;
    config_ = nullptr;
    dummy_node_ = nullptr;
  }

  void SetUp()
  {
    node_ = new nav2_behavior_tree::DistanceController("distance_controller", *config_);
    node_->setChild(dummy_node_);
  }

  void TearDown()
  {
    node_ = nullptr;
  }

protected:
  static nav2_behavior_tree::TransformHandler * transform_handler_;
  static BT::NodeConfiguration * config_;
  static nav2_behavior_tree::DistanceController * node_;
  static nav2_behavior_tree::DummyNode * dummy_node_;
};

nav2_behavior_tree::TransformHandler * DistanceControllerTestFixture::transform_handler_ = nullptr;
BT::NodeConfiguration * DistanceControllerTestFixture::config_ = nullptr;
nav2_behavior_tree::DistanceController * DistanceControllerTestFixture::node_ = nullptr;
nav2_behavior_tree::DummyNode * DistanceControllerTestFixture::dummy_node_ = nullptr;

TEST_F(DistanceControllerTestFixture, test_initial_status_is_idle)
{
  EXPECT_EQ(node_->status(), BT::NodeStatus::IDLE);
}

TEST_F(DistanceControllerTestFixture, test_failure_when_child_is_idle)
{
  dummy_node_->setStatus(BT::NodeStatus::IDLE);
  EXPECT_EQ(node_->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(DistanceControllerTestFixture, test_failure_when_child_fails)
{
  dummy_node_->setStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(node_->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(DistanceControllerTestFixture, test_running_when_child_is_running)
{
  dummy_node_->setStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(node_->executeTick(), BT::NodeStatus::RUNNING);
}

TEST_F(DistanceControllerTestFixture, test_behavior)
{
  EXPECT_EQ(node_->status(), BT::NodeStatus::IDLE);

  dummy_node_->setStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;
  pose.pose.orientation.w = 1;

  double traveled = 0;

  transform_handler_->updateRobotPose(pose.pose);

  // Wait for transforms to actually update
  while (traveled < 0.1) {
    if (nav2_util::getCurrentPose(pose, *transform_handler_->getBuffer())) {
      traveled = std::sqrt(
        pose.pose.position.x * pose.pose.position.x + pose.pose.position.y * pose.pose.position.y);
    }
    EXPECT_EQ(node_->executeTick(), BT::NodeStatus::RUNNING);
  }

  for (int i = 1; i < 10; i++) {
    pose.pose.position.x = i * 1.1;
    transform_handler_->updateRobotPose(pose.pose);

    // Wait for transforms to actually update
    while (traveled < i) {
      if (nav2_util::getCurrentPose(pose, *transform_handler_->getBuffer())) {
        traveled = std::sqrt(
          pose.pose.position.x * pose.pose.position.x +
          pose.pose.position.y * pose.pose.position.y);
      }
    }

    dummy_node_->setStatus(BT::NodeStatus::SUCCESS);

    EXPECT_EQ(node_->executeTick(), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);
  }
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
