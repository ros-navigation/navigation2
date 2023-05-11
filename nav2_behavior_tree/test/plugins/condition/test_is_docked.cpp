// Copyright (c) 2023 Alberto J. Tudela Roldán
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
#include <chrono>

#include "nav2_msgs/msg/dock_state.hpp"

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/is_docked_condition.hpp"

class IsDockedConditionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("test_is_docked");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);

    factory_->registerNodeType<nav2_behavior_tree::IsDockedCondition>("IsDocked");

    dock_state_pub_ = node_->create_publisher<nav2_msgs::msg::DockState>(
      "/dock_status",
      rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    dock_state_pub_.reset();
    node_.reset();
    factory_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static rclcpp::Publisher<nav2_msgs::msg::DockState>::SharedPtr dock_state_pub_;
};

rclcpp::Node::SharedPtr IsDockedConditionTestFixture::node_ = nullptr;
BT::NodeConfiguration * IsDockedConditionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> IsDockedConditionTestFixture::factory_ = nullptr;
rclcpp::Publisher<nav2_msgs::msg::DockState>::SharedPtr
IsDockedConditionTestFixture::dock_state_pub_ = nullptr;

TEST_F(IsDockedConditionTestFixture, test_behavior_dock_status)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <IsDocked dock_state_topic="/dock_status"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  nav2_msgs::msg::DockState dock_state_msg;
  dock_state_msg.docked = true;
  dock_state_pub_->publish(dock_state_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node_);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  dock_state_msg.docked = false;
  dock_state_pub_->publish(dock_state_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node_);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
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
