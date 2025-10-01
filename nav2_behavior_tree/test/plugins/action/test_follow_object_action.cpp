// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
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

#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/follow_object_action.hpp"

class FollowObjectActionServer
  : public TestActionServer<nav2_msgs::action::FollowObject>
{
public:
  FollowObjectActionServer()
  : TestActionServer("follow_object")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::FollowObject>> goal_handle)
  override
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::FollowObject::Result>();
    goal_handle->succeed(result);
  }
};

class FollowObjectActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("follow_object_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<nav2::LifecycleNode::SharedPtr>(
      "node", node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout", std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration", std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout", std::chrono::milliseconds(1000));
    config_->blackboard->set<bool>("initial_pose_received", false);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::FollowObjectAction>(
          name, "follow_object", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::FollowObjectAction>("FollowObject", builder);
  }

  static void TearDownTestCase()
  {
    factory_.reset();
    action_server_.reset();
    delete config_;
    config_ = nullptr;
    node_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<FollowObjectActionServer> action_server_;

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

nav2::LifecycleNode::SharedPtr FollowObjectActionTestFixture::node_ = nullptr;
std::shared_ptr<FollowObjectActionServer>
FollowObjectActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * FollowObjectActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> FollowObjectActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> FollowObjectActionTestFixture::tree_ = nullptr;

TEST_F(FollowObjectActionTestFixture, test_ports)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <FollowObject />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<double>("max_duration"), 0.0);

  xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <FollowObject pose_topic="test_topic" max_duration="20.0" tracked_frame="test_frame"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("pose_topic"), "test_topic");
  EXPECT_EQ(tree_->rootNode()->getInput<double>("max_duration"), 20.0);
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("tracked_frame"), "test_frame");
}

TEST_F(FollowObjectActionTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <FollowObject pose_topic="test_topic"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  geometry_msgs::msg::PoseStamped pose;

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // the goal should have reached our server
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  // halt node so another goal can be sent
  tree_->haltTree();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and spin on new thread
  FollowObjectActionTestFixture::action_server_ =
    std::make_shared<FollowObjectActionServer>();

  std::thread server_thread([]() {
      rclcpp::spin(FollowObjectActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
