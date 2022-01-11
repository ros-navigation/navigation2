// Copyright (c) 2022 Pradheep Padmanabhan - Neobotix GmbH
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

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_service.hpp"
#include "nav2_behavior_tree/plugins/action/controller_cancel_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

class ChangeStateService : public TestService<lifecycle_msgs::srv::ChangeState>
{
public:
  ChangeStateService()
  : TestService("/controller_server/change_state")
  {}
};

class CancelControlTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("controller_cancel_test_fixture");
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

    factory_->registerNodeType<nav2_behavior_tree::ControllerCancel>("CancelControl");
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    server_.reset();
    node_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<ChangeStateService> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;

  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr CancelControlTestFixture::node_ = nullptr;

BT::NodeConfiguration * CancelControlTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> CancelControlTestFixture::factory_ = nullptr;
std::shared_ptr<ChangeStateService>
CancelControlTestFixture::server_ = nullptr;
std::shared_ptr<BT::Tree> CancelControlTestFixture::tree_ = nullptr;

TEST_F(CancelControlTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <CancelControl name="ControlCancel" service_name="/controller_server/change_state"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  CancelControlTestFixture::server_ = std::make_shared<ChangeStateService>();
  std::thread server_thread([]() {
      rclcpp::spin(CancelControlTestFixture::server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();


  return all_successful;
}
