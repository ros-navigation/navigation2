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

#include "utils/test_action_server.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/plugins/action/progress_checker_selector_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"

class ProgressCheckerSelectorTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("progress_checker_selector_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set("node", node_);

    BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<nav2_behavior_tree::ProgressCheckerSelector>(name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::ProgressCheckerSelector>(
      "ProgressCheckerSelector",
      builder);
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

rclcpp::Node::SharedPtr ProgressCheckerSelectorTestFixture::node_ = nullptr;

BT::NodeConfiguration * ProgressCheckerSelectorTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ProgressCheckerSelectorTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ProgressCheckerSelectorTestFixture::tree_ = nullptr;

TEST_F(ProgressCheckerSelectorTestFixture, test_custom_topic)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <ProgressCheckerSelector selected_progress_checker="{selected_progress_checker}" default_progress_checker="SimpleProgressCheck" topic_name="progress_checker_selector_custom_topic_name"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // check default value
  std::string selected_progress_checker_result;
  [[maybe_unused]] auto res = config_->blackboard->get(
    "selected_progress_checker",
    selected_progress_checker_result);

  EXPECT_EQ(selected_progress_checker_result, "SimpleProgressCheck");

  std_msgs::msg::String selected_progress_checker_cmd;

  selected_progress_checker_cmd.data = "AngularProgressChecker";

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  auto progress_checker_selector_pub =
    node_->create_publisher<std_msgs::msg::String>(
    "progress_checker_selector_custom_topic_name", qos);

  // publish a few updates of the selected_progress_checker
  auto start = node_->now();
  while ((node_->now() - start).seconds() < 0.5) {
    tree_->rootNode()->executeTick();
    progress_checker_selector_pub->publish(selected_progress_checker_cmd);

    rclcpp::spin_some(node_);
  }

  // check progress_checker updated
  res = config_->blackboard->get("selected_progress_checker", selected_progress_checker_result);
  EXPECT_EQ("AngularProgressChecker", selected_progress_checker_result);
}

TEST_F(ProgressCheckerSelectorTestFixture, test_default_topic)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <ProgressCheckerSelector selected_progress_checker="{selected_progress_checker}" default_progress_checker="GridBased"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // check default value
  std::string selected_progress_checker_result;
  [[maybe_unused]] auto res = config_->blackboard->get(
    "selected_progress_checker",
    selected_progress_checker_result);

  EXPECT_EQ(selected_progress_checker_result, "GridBased");

  std_msgs::msg::String selected_progress_checker_cmd;

  selected_progress_checker_cmd.data = "RRT";

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  auto progress_checker_selector_pub =
    node_->create_publisher<std_msgs::msg::String>("progress_checker_selector", qos);

  // publish a few updates of the selected_progress_checker
  auto start = node_->now();
  while ((node_->now() - start).seconds() < 0.5) {
    tree_->rootNode()->executeTick();
    progress_checker_selector_pub->publish(selected_progress_checker_cmd);

    rclcpp::spin_some(node_);
  }

  // check goal_checker updated
  res = config_->blackboard->get("selected_progress_checker", selected_progress_checker_result);
  EXPECT_EQ("RRT", selected_progress_checker_result);
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
