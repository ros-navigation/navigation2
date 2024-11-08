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
#include "nav2_behavior_tree/plugins/decorator/goal_updated_controller.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class GoalUpdatedControllerTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    // Setting fake goals on blackboard
    geometry_msgs::msg::PoseStamped goal1;
    goal1.header.stamp = node_->now();
    std::vector<geometry_msgs::msg::PoseStamped> poses1;
    poses1.push_back(goal1);
    config_->blackboard->set("goal", goal1);
    config_->blackboard->set("goals", poses1);
    bt_node_ = std::make_shared<nav2_behavior_tree::GoalUpdatedController>(
      "goal_updated_controller", *config_);
    dummy_node_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    bt_node_->setChild(dummy_node_.get());
  }

  void TearDown()
  {
    dummy_node_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::GoalUpdatedController> bt_node_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> dummy_node_;
};

std::shared_ptr<nav2_behavior_tree::GoalUpdatedController>
GoalUpdatedControllerTestFixture::bt_node_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
GoalUpdatedControllerTestFixture::dummy_node_ = nullptr;

TEST_F(GoalUpdatedControllerTestFixture, test_behavior)
{
  // Creating updated fake-goals
  geometry_msgs::msg::PoseStamped goal2;
  goal2.header.stamp = node_->now();
  std::vector<geometry_msgs::msg::PoseStamped> poses2;
  poses2.push_back(goal2);

  // starting in idle
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);

  // tick for the first time, dummy node should be ticked
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);

  // tick again with updated goal, dummy node should be ticked
  config_->blackboard->set("goal", goal2);
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);

  // tick again without update, dummy node should not be ticked
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);

  // tick again with updated goals, dummy node should be ticked
  config_->blackboard->set("goals", poses2);
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);
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
