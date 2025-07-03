// Copyright (c) 2025 Polymath Robotics
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
#include <memory>

#include "utils/test_behavior_tree_fixture.hpp"
#include "utils/test_dummy_tree_node.hpp"
#include "nav2_behavior_tree/plugins/control/nonblocking_sequence.hpp"

class NonblockingSequenceTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp() override
  {
    bt_node_ = std::make_shared<nav2_behavior_tree::NonblockingSequence>(
      "nonblocking_sequence", *config_);
    first_child_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    second_child_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    third_child_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    bt_node_->addChild(first_child_.get());
    bt_node_->addChild(second_child_.get());
    bt_node_->addChild(third_child_.get());
  }

  void TearDown() override
  {
    first_child_.reset();
    second_child_.reset();
    third_child_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::NonblockingSequence> bt_node_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> first_child_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> second_child_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> third_child_;
};

std::shared_ptr<nav2_behavior_tree::NonblockingSequence>
NonblockingSequenceTestFixture::bt_node_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
NonblockingSequenceTestFixture::first_child_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
NonblockingSequenceTestFixture::second_child_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
NonblockingSequenceTestFixture::third_child_ = nullptr;

TEST_F(NonblockingSequenceTestFixture, test_failure_on_idle_child)
{
  first_child_->changeStatus(BT::NodeStatus::IDLE);
  EXPECT_THROW(bt_node_->executeTick(), std::runtime_error);
}

TEST_F(NonblockingSequenceTestFixture, test_failure)
{
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(third_child_->status(), BT::NodeStatus::IDLE);

  first_child_->changeStatus(BT::NodeStatus::SUCCESS);
  second_child_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(third_child_->status(), BT::NodeStatus::IDLE);

  first_child_->changeStatus(BT::NodeStatus::SUCCESS);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  third_child_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(third_child_->status(), BT::NodeStatus::IDLE);

  first_child_->changeStatus(BT::NodeStatus::SUCCESS);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  third_child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(third_child_->status(), BT::NodeStatus::IDLE);
}

TEST_F(NonblockingSequenceTestFixture, test_behavior)
{
  first_child_->changeStatus(BT::NodeStatus::RUNNING);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  third_child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);

  first_child_->changeStatus(BT::NodeStatus::SUCCESS);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  third_child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);

  first_child_->changeStatus(BT::NodeStatus::RUNNING);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  third_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);

  first_child_->changeStatus(BT::NodeStatus::SUCCESS);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  third_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // Even if first two children are running, we will still tick the third
  // node, which if set to failure, fails everything
  first_child_->changeStatus(BT::NodeStatus::RUNNING);
  second_child_->changeStatus(BT::NodeStatus::RUNNING);
  third_child_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(third_child_->status(), BT::NodeStatus::IDLE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
