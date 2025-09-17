// Copyright (c) 2025 David Grbac
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

#include "utils/test_service.hpp"
#include "nav2_behavior_tree/plugins/action/toggle_collision_monitor_service.hpp"

class ToggleCollisionMonitorService : public TestService<nav2_msgs::srv::Toggle>
{
public:
  ToggleCollisionMonitorService()
  : TestService("toggle_collision_monitor")
  {}
};

class ToggleCollisionMonitorTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("toggle_collision_monitor_test_fixture");
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

    factory_->registerNodeType<nav2_behavior_tree::ToggleCollisionMonitorService>(
      "ToggleCollisionMonitor");
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    server_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<ToggleCollisionMonitorService> server_;

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

nav2::LifecycleNode::SharedPtr ToggleCollisionMonitorTestFixture::node_ = nullptr;
std::shared_ptr<ToggleCollisionMonitorService>
ToggleCollisionMonitorTestFixture::server_ = nullptr;
BT::NodeConfiguration * ToggleCollisionMonitorTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory>
ToggleCollisionMonitorTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ToggleCollisionMonitorTestFixture::tree_ = nullptr;

TEST_F(ToggleCollisionMonitorTestFixture, test_tick_enable_collision_monitor)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <ToggleCollisionMonitor service_name="toggle_collision_monitor" enable="true"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
}

TEST_F(ToggleCollisionMonitorTestFixture, test_tick_disable_collision_monitor)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <ToggleCollisionMonitor service_name="toggle_collision_monitor" enable="false"/>
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
  ToggleCollisionMonitorTestFixture::server_ =
    std::make_shared<ToggleCollisionMonitorService>();
  std::thread server_thread([]() {
      rclcpp::spin(ToggleCollisionMonitorTestFixture::server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
