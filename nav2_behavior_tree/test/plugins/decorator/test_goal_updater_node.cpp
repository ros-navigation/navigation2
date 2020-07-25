// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_action_server.hpp"
#include "nav2_behavior_tree/plugins/decorator/goal_updater_node.hpp"


class GoalUpdaterTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("goal_updater_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::GoalUpdater>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::GoalUpdater>(
      "GoalUpdater", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr GoalUpdaterTestFixture::node_ = nullptr;

BT::NodeConfiguration * GoalUpdaterTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> GoalUpdaterTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> GoalUpdaterTestFixture::tree_ = nullptr;

TEST_F(GoalUpdaterTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <GoalUpdater input_goal="{goal}" output_goal="{updated_goal}">
            <AlwaysSuccess/>
          </GoalUpdater>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new goal and set it on blackboard
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->now();
  goal.pose.position.x = 1.0;
  config_->blackboard->set("goal", goal);

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  geometry_msgs::msg::PoseStamped updated_goal;
  config_->blackboard->get("updated_goal", updated_goal);

  EXPECT_EQ(updated_goal, goal);

  geometry_msgs::msg::PoseStamped goal_to_update;
  goal_to_update.header.stamp = node_->now();
  goal_to_update.pose.position.x = 2.0;

  auto goal_updater_pub =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", 10);

  auto start = node_->now();
  while ((node_->now() - start).seconds() < 0.5) {
    tree_->rootNode()->executeTick();
    goal_updater_pub->publish(goal_to_update);

    rclcpp::spin_some(node_);
  }

  config_->blackboard->get("updated_goal", updated_goal);

  EXPECT_NE(updated_goal, goal);
  EXPECT_EQ(updated_goal, goal_to_update);
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
