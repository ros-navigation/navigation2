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
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "utils/test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/compute_path_to_pose_action.hpp"

class ComputePathToPoseActionServer : public TestActionServer<nav2_msgs::action::ComputePathToPose>
{
public:
  ComputePathToPoseActionServer()
  : TestActionServer("compute_path_to_pose")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::ComputePathToPose>> goal_handle)
  override
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::ComputePathToPose::Result>();
    result->path.poses.resize(2);
    result->path.poses[1].pose.position.x = goal->goal.pose.position.x;
    if (goal->use_start) {
      result->path.poses[0].pose.position.x = goal->start.pose.position.x;
    } else {
      result->path.poses[0].pose.position.x = 0.0;
    }
    goal_handle->succeed(result);
  }
};

class ComputePathToPoseActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("compute_path_to_pose_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout",
      std::chrono::milliseconds(1000));
    config_->blackboard->set("initial_pose_received", false);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::ComputePathToPoseAction>(
          name, "compute_path_to_pose", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::ComputePathToPoseAction>(
      "ComputePathToPose", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    action_server_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<ComputePathToPoseActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr ComputePathToPoseActionTestFixture::node_ = nullptr;
std::shared_ptr<ComputePathToPoseActionServer>
ComputePathToPoseActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * ComputePathToPoseActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ComputePathToPoseActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ComputePathToPoseActionTestFixture::tree_ = nullptr;

TEST_F(ComputePathToPoseActionTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
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

  // the goal should have reached our server
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("planner_id"), std::string("GridBased"));
  EXPECT_EQ(action_server_->getCurrentGoal()->goal.pose.position.x, 1.0);
  EXPECT_FALSE(action_server_->getCurrentGoal()->use_start);
  EXPECT_EQ(action_server_->getCurrentGoal()->planner_id, std::string("GridBased"));

  // check if returned path is correct
  nav_msgs::msg::Path path;
  EXPECT_TRUE(config_->blackboard->get<nav_msgs::msg::Path>("path", path));
  EXPECT_EQ(path.poses.size(), 2u);
  EXPECT_EQ(path.poses[0].pose.position.x, 0.0);
  EXPECT_EQ(path.poses[1].pose.position.x, 1.0);

  // halt node so another goal can be sent
  tree_->haltTree();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);

  // set new goal
  goal.pose.position.x = -2.5;
  config_->blackboard->set("goal", goal);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action_server_->getCurrentGoal()->goal.pose.position.x, -2.5);

  EXPECT_TRUE(config_->blackboard->get<nav_msgs::msg::Path>("path", path));
  EXPECT_EQ(path.poses.size(), 2u);
  EXPECT_EQ(path.poses[0].pose.position.x, 0.0);
  EXPECT_EQ(path.poses[1].pose.position.x, -2.5);
}

TEST_F(ComputePathToPoseActionTestFixture, test_tick_use_start)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <ComputePathToPose goal="{goal}" start="{start}" path="{path}" planner_id="GridBased"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new start and set it on blackboard
  geometry_msgs::msg::PoseStamped start;
  start.header.stamp = node_->now();
  start.pose.position.x = 2.0;
  config_->blackboard->set("start", start);

  // create new goal and set it on blackboard
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->now();
  goal.pose.position.x = 1.0;
  config_->blackboard->set("goal", goal);

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // the goal should have reached our server
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("planner_id"), std::string("GridBased"));
  EXPECT_EQ(action_server_->getCurrentGoal()->goal.pose.position.x, 1.0);
  EXPECT_EQ(action_server_->getCurrentGoal()->start.pose.position.x, 2.0);
  EXPECT_TRUE(action_server_->getCurrentGoal()->use_start);
  EXPECT_EQ(action_server_->getCurrentGoal()->planner_id, std::string("GridBased"));

  // check if returned path is correct
  nav_msgs::msg::Path path;
  EXPECT_TRUE(config_->blackboard->get<nav_msgs::msg::Path>("path", path));
  EXPECT_EQ(path.poses.size(), 2u);
  EXPECT_EQ(path.poses[0].pose.position.x, 2.0);
  EXPECT_EQ(path.poses[1].pose.position.x, 1.0);

  // halt node so another goal can be sent
  tree_->haltTree();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);

  // set new goal and new start
  goal.pose.position.x = -2.5;
  start.pose.position.x = -1.5;
  config_->blackboard->set("goal", goal);
  config_->blackboard->set("start", start);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action_server_->getCurrentGoal()->goal.pose.position.x, -2.5);

  EXPECT_TRUE(config_->blackboard->get<nav_msgs::msg::Path>("path", path));
  EXPECT_EQ(path.poses.size(), 2u);
  EXPECT_EQ(path.poses[0].pose.position.x, -1.5);
  EXPECT_EQ(path.poses[1].pose.position.x, -2.5);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and spin on new thread
  ComputePathToPoseActionTestFixture::action_server_ =
    std::make_shared<ComputePathToPoseActionServer>();

  std::thread server_thread([]() {
      rclcpp::spin(ComputePathToPoseActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
