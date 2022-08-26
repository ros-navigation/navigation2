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

#ifndef TEST_BEHAVIOR_TREE_FIXTURE_HPP_
#define TEST_BEHAVIOR_TREE_FIXTURE_HPP_

#include <gtest/gtest.h>
#include <memory>
#include <set>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "test_transform_handler.hpp"
#include "test_dummy_tree_node.hpp"

namespace nav2_behavior_tree
{

class BehaviorTreeTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("test_behavior_tree_fixture");
    transform_handler_ = std::make_shared<nav2_behavior_tree::TransformHandler>(node_);
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);
    config_->blackboard->set<std::shared_ptr<tf2_ros::Buffer>>(
      "tf_buffer",
      transform_handler_->getBuffer());
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<bool>("initial_pose_received", false);

    transform_handler_->activate();
    transform_handler_->waitForTransform();
  }

  static void TearDownTestCase()
  {
    transform_handler_->deactivate();
    delete config_;
    config_ = nullptr;
    transform_handler_.reset();
    node_.reset();
    factory_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<nav2_behavior_tree::TransformHandler> transform_handler_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
};

}  // namespace nav2_behavior_tree

rclcpp::Node::SharedPtr nav2_behavior_tree::BehaviorTreeTestFixture::node_ = nullptr;

std::shared_ptr<nav2_behavior_tree::TransformHandler>
nav2_behavior_tree::BehaviorTreeTestFixture::transform_handler_ = nullptr;

BT::NodeConfiguration * nav2_behavior_tree::BehaviorTreeTestFixture::config_ = nullptr;

std::shared_ptr<BT::BehaviorTreeFactory>
nav2_behavior_tree::BehaviorTreeTestFixture::factory_ = nullptr;

#endif  // TEST_BEHAVIOR_TREE_FIXTURE_HPP_
