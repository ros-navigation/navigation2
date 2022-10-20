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
#include <chrono>
#include <string>

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/transform_available_condition.hpp"

class TransformAvailableConditionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("test_behavior_tree_fixture");
    transform_handler_ = std::make_shared<nav2_behavior_tree::TransformHandler>(node_);
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);
    config_->blackboard->set<std::shared_ptr<tf2_ros::Buffer>>(
      "tf_buffer",
      transform_handler_->getBuffer());
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<bool>("initial_pose_received", false);
  }

  static void TearDownTestCase()
  {
    transform_handler_->deactivate();
    delete config_;
    config_ = nullptr;
    transform_handler_.reset();
    node_.reset();
    factory_.reset();
  }

  void SetUp()
  {
    std::string xml_txt =
      R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <TransformAvailable child="base_link" parent="map" />
        </BehaviorTree>
      </root>)";

    factory_->registerNodeType<nav2_behavior_tree::TransformAvailableCondition>(
      "TransformAvailable");
    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  }

  void TearDown()
  {
    tree_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<nav2_behavior_tree::TransformHandler> transform_handler_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr TransformAvailableConditionTestFixture::node_ = nullptr;
std::shared_ptr<nav2_behavior_tree::TransformHandler>
TransformAvailableConditionTestFixture::transform_handler_ = nullptr;
BT::NodeConfiguration * TransformAvailableConditionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory>
TransformAvailableConditionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> TransformAvailableConditionTestFixture::tree_ = nullptr;

TEST_F(TransformAvailableConditionTestFixture, test_behavior)
{
  EXPECT_EQ(tree_->tickRoot(), BT::NodeStatus::FAILURE);
  transform_handler_->activate();
  transform_handler_->waitForTransform();
  EXPECT_EQ(tree_->tickRoot(), BT::NodeStatus::SUCCESS);
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
