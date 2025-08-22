// Copyright (c) 2025 Enjoy Robotics
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
#include "utils/get_node_from_tree.hpp"
#include "nav2_behavior_tree/plugins/control/persistent_sequence.hpp"

class PersistentSequenceTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  static void SetUpTestCase()
  {
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();
    config_ = new BT::NodeConfiguration();
    config_->blackboard = BT::Blackboard::create();
    config_->blackboard->set("seq_child_idx", 0);

    factory_->registerNodeType<nav2_behavior_tree::PersistentSequenceNode>(
      "PersistentSequence");

    // Register dummy node for testing
    factory_->registerNodeType<nav2_behavior_tree::DummyNode>("DummyNode");
  }

  static void TearDownTestCase()
  {
    if (config_) {
      delete config_;
      config_ = nullptr;
    }
    tree_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

BT::NodeConfiguration * PersistentSequenceTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory>
PersistentSequenceTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> PersistentSequenceTestFixture::tree_ = nullptr;

TEST_F(PersistentSequenceTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <PersistentSequence current_child_idx="{seq_child_idx}">
            <AlwaysSuccess/>
          </PersistentSequence>
        </BehaviorTree>
      </root>)";
  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  config_->blackboard->set("seq_child_idx", 0);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
}

TEST_F(PersistentSequenceTestFixture, test_behavior)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <PersistentSequence current_child_idx="{seq_child_idx}">
            <DummyNode/>
            <DummyNode/>
            <DummyNode/>
          </PersistentSequence>
        </BehaviorTree>
      </root>)";
  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // get dummy nodes so we can change their status
  auto child0 =
    nav2_behavior_tree::get_node_from_tree<nav2_behavior_tree::DummyNode>(tree_, 0);
  auto child1 =
    nav2_behavior_tree::get_node_from_tree<nav2_behavior_tree::DummyNode>(tree_, 1);
  auto child2 =
    nav2_behavior_tree::get_node_from_tree<nav2_behavior_tree::DummyNode>(tree_, 2);
  ASSERT_NE(child0, nullptr);
  ASSERT_NE(child1, nullptr);
  ASSERT_NE(child2, nullptr);
  child0->changeStatus(BT::NodeStatus::RUNNING);
  child1->changeStatus(BT::NodeStatus::RUNNING);
  child2->changeStatus(BT::NodeStatus::RUNNING);

  // Set child index to 0 and child0 to RUNNING, expect RUNNING and idx = 0
  config_->blackboard->set("seq_child_idx", 0);
  child0->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(config_->blackboard->get<int>("seq_child_idx"), 0);

  // Set child0 to SUCCESS, expect SUCCESS and idx = 1
  child0->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(config_->blackboard->get<int>("seq_child_idx"), 1);

  // Set child1 to FAILURE, expect FAILURE and idx = 0
  child1->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(config_->blackboard->get<int>("seq_child_idx"), 0);

  // Set idx to 1, child0 to FAILURE, child1 to SKIPPED and child2 to RUNNING,
  // expect RUNNING and idx = 2
  config_->blackboard->set("seq_child_idx", 1);
  child0->changeStatus(BT::NodeStatus::FAILURE);
  child1->changeStatus(BT::NodeStatus::SKIPPED);
  child2->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(config_->blackboard->get<int>("seq_child_idx"), 2);

  // Set child2 to SUCCESS, expect SUCCESS and idx = 0
  child2->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(config_->blackboard->get<int>("seq_child_idx"), 0);
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
