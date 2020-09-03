// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2019 Intel Corporation
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

#include "std_msgs/msg/float64.hpp"

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/is_battery_low_condition.hpp"

class IsBatteryLowConditionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("test_is_battery_low");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);

    factory_->registerNodeType<nav2_behavior_tree::IsBatteryLowCondition>("IsBatteryLow");

    battery_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/battery_status",
      rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    battery_pub_.reset();
    node_.reset();
    factory_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_pub_;
};

rclcpp::Node::SharedPtr IsBatteryLowConditionTestFixture::node_ = nullptr;
BT::NodeConfiguration * IsBatteryLowConditionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> IsBatteryLowConditionTestFixture::factory_ = nullptr;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
IsBatteryLowConditionTestFixture::battery_pub_ = nullptr;

TEST_F(IsBatteryLowConditionTestFixture, test_behavior)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <IsBatteryLow min_battery="50.0" battery_topic="/battery_status"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  std_msgs::msg::Float64 battery_msg;
  battery_msg.data = 100.0;
  battery_pub_->publish(battery_msg);
  rclcpp::spin_some(node_);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

  battery_msg.data = 50.0;
  battery_pub_->publish(battery_msg);
  rclcpp::spin_some(node_);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  battery_msg.data = 51.0;
  battery_pub_->publish(battery_msg);
  rclcpp::spin_some(node_);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  battery_msg.data = 0.0;
  battery_pub_->publish(battery_msg);
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
