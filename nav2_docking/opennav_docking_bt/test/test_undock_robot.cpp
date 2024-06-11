// Copyright (c) 2024 Open Navigation LLC
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

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "opennav_docking_bt/undock_robot.hpp"

class UndockRobotActionServer
  : public TestActionServer<nav2_msgs::action::UndockRobot>
{
public:
  UndockRobotActionServer()
  : TestActionServer("undock_robot")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::UndockRobot>>
    goal_handle)
  override
  {
    const auto goal = goal_handle->get_goal();
    auto result =
      std::make_shared<nav2_msgs::action::UndockRobot::Result>();
    result->success = true;
    goal_handle->succeed(result);
  }
};

class UndockRobotActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("undock_robot_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
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
    config_->blackboard->set<bool>("initial_pose_received", false);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<opennav_docking_bt::UndockRobotAction>(
          name, "undock_robot", config);
      };

    factory_->registerBuilder<opennav_docking_bt::UndockRobotAction>(
      "UndockRobot", builder);
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

  static std::shared_ptr<UndockRobotActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr UndockRobotActionTestFixture::node_ = nullptr;
std::shared_ptr<UndockRobotActionServer>
UndockRobotActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * UndockRobotActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> UndockRobotActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> UndockRobotActionTestFixture::tree_ = nullptr;

TEST_F(UndockRobotActionTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <UndockRobot dock_type="dock1"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

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
  UndockRobotActionTestFixture::action_server_ =
    std::make_shared<UndockRobotActionServer>();

  std::thread server_thread([]() {
      rclcpp::spin(UndockRobotActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
