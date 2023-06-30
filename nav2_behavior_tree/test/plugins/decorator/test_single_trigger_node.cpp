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

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/decorator/single_trigger_node.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

// Shim BT node to access protected resetStatus method
class ShimNode : public nav2_behavior_tree::SingleTrigger
{
public:
  ShimNode(
    const std::string & name,
    const BT::NodeConfiguration & confi)
  : SingleTrigger(name, confi)
  {}
  ~ShimNode() {}
  void changeStatus() {resetStatus();}
};

class SingleTriggerTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    bt_node_ = std::make_shared<ShimNode>("single_trigger", *config_);
    dummy_node_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    bt_node_->setChild(dummy_node_.get());
  }

  void TearDown()
  {
    dummy_node_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<ShimNode> bt_node_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> dummy_node_;
};

std::shared_ptr<ShimNode>
SingleTriggerTestFixture::bt_node_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
SingleTriggerTestFixture::dummy_node_ = nullptr;

TEST_F(SingleTriggerTestFixture, test_behavior)
{
  // starting in idle
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);

  // tick once, should work
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);

  // tick again with dummy node success, should fail
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // tick again with dummy node idle, should still fail
  dummy_node_->changeStatus(BT::NodeStatus::IDLE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // halt BT for a new execution run, should work when dummy node is running
  // and once when dummy node returns success and then fail
  bt_node_->halt();
  bt_node_->changeStatus();  // BTv3.8+ doesn't reset root node automatically
  dummy_node_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
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
