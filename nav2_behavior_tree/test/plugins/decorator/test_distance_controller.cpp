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

#include "../../test_transform_handler.hpp"
#include "../../test_dummy_tree_node.hpp"
#include "nav2_behavior_tree/plugins/distance_controller.hpp"

class DistanceControllerTestFixture : public ::testing::Test
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
    node_ = new nav2_behavior_tree::DistanceController("distance_controller", *config_);
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
  static nav2_behavior_tree::DistanceController * node_;
  static nav2_behavior_tree::DummyNode * dummy_node_;
};

std::shared_ptr<nav2_behavior_tree::TransformHandler>
DistanceControllerTestFixture::transform_handler_ = nullptr;
std::shared_ptr<BT::NodeConfiguration> DistanceControllerTestFixture::config_ = nullptr;
nav2_behavior_tree::DistanceController * DistanceControllerTestFixture::node_ = nullptr;
nav2_behavior_tree::DummyNode * DistanceControllerTestFixture::dummy_node_ = nullptr;

TEST_F(DistanceControllerTestFixture, test_behavior)
{
  EXPECT_EQ(node_->status(), BT::NodeStatus::IDLE);

  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.orientation.w = 1;

  double traveled = 0;
  for (int i = 1; i <= 20; i++) {
    pose.pose.position.x = i * 0.51;
    transform_handler_->updateRobotPose(pose.pose);

    // Wait for transforms to actually update
    // updated pose is i * 0.55
    // we wait fot the traveled distance to reach a value > i * 0.5
    // we can assume the current transform has been updated at this point
    while (traveled < i * 0.5) {
      if (nav2_util::getCurrentPose(pose, *transform_handler_->getBuffer())) {
        traveled = std::sqrt(
          pose.pose.position.x * pose.pose.position.x +
          pose.pose.position.y * pose.pose.position.y);
      }
    }

    dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);

    if (i % 2) {
      EXPECT_EQ(node_->executeTick(), BT::NodeStatus::RUNNING);
    } else {
      EXPECT_EQ(node_->executeTick(), BT::NodeStatus::SUCCESS);
      EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);
    }
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
