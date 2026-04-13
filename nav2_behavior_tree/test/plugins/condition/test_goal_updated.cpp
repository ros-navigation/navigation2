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
#include <memory>
#include <set>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/goal_updated_condition.hpp"

class GoalUpdatedConditionTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    config_->input_ports["goals"] = "";
    config_->input_ports["goal"] = "";
    bt_node_ = std::make_shared<nav2_behavior_tree::GoalUpdatedCondition>(
      "goal_updated", *config_);
  }

  void TearDown()
  {
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::GoalUpdatedCondition> bt_node_;
};

std::shared_ptr<nav2_behavior_tree::GoalUpdatedCondition>
GoalUpdatedConditionTestFixture::bt_node_ = nullptr;

class GoalUpdatedConditionGlobalTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    config_->input_ports["goals"] = "";
    config_->input_ports["goal"] = "";
    config_->input_ports["is_global"] = "true";
    bt_node_ = std::make_shared<nav2_behavior_tree::GoalUpdatedCondition>(
      "goal_updated", *config_);
  }
  void TearDown()
  {
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::GoalUpdatedCondition> bt_node_;
};
std::shared_ptr<nav2_behavior_tree::GoalUpdatedCondition>
GoalUpdatedConditionGlobalTestFixture::bt_node_ = nullptr;

TEST_F(GoalUpdatedConditionTestFixture, test_behavior)
{
  geometry_msgs::msg::PoseStamped goal;
  config_->blackboard->set("goal", goal);

  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  goal.pose.position.x = 1.0;
  config_->blackboard->set("goal", goal);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(GoalUpdatedConditionGlobalTestFixture, test_runid_global_mode)
{
  config_->blackboard->set<std::string>("run_id", "runid_1");

  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = 0.0;
  config_->blackboard->set("goal", goal);

  // First tick: snapshot captured, goal unchanged -> FAILURE
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // Same run, goal changes -> SUCCESS
  goal.pose.position.x = 1.0;
  config_->blackboard->set("goal", goal);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // Same run, goal unchanged -> FAILURE
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // New RunID with same goal: old snapshot kept, no change -> FAILURE
  config_->blackboard->set<std::string>("run_id", "runid_2");
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // New RunID without changing goal: old snapshot kept, no change -> FAILURE
  config_->blackboard->set<std::string>("run_id", "runid_3");
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // Now change the goal on the same run: detected as update -> SUCCESS
  goal.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // Same run, goal unchanged -> FAILURE
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
