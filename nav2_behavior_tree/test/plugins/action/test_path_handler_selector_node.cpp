// Copyright (c) 2025 Maurice Alexander Purnawan
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

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/plugins/action/path_handler_selector_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"

class PathHandlerSelectorTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("path_handler_selector_test_fixture");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());

    // Configure and activate the lifecycle node
    node_->configure();
    node_->activate();

    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));

    BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<nav2_behavior_tree::PathHandlerSelector>(name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::PathHandlerSelector>(
      "PathHandlerSelector",
      builder);
  }

  static void TearDownTestCase()
  {
    // Properly deactivate and cleanup the lifecycle node
    node_->deactivate();
    node_->cleanup();

    delete config_;
    config_ = nullptr;
    node_.reset();
    factory_.reset();
    executor_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

nav2::LifecycleNode::SharedPtr PathHandlerSelectorTestFixture::node_ = nullptr;
rclcpp::executors::SingleThreadedExecutor::SharedPtr PathHandlerSelectorTestFixture::executor_ =
  nullptr;

BT::NodeConfiguration * PathHandlerSelectorTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> PathHandlerSelectorTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> PathHandlerSelectorTestFixture::tree_ = nullptr;

TEST_F(PathHandlerSelectorTestFixture, test_custom_topic)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <PathHandlerSelector selected_path_handler="{selected_path_handler}" default_path_handler="SimplePathHandler" topic_name="path_handler_selector_custom_topic_name"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // check default value
  std::string selected_path_handler_result;
  EXPECT_TRUE(config_->blackboard->get("selected_path_handler", selected_path_handler_result));

  EXPECT_EQ(selected_path_handler_result, "SimplePathHandler");

  std_msgs::msg::String selected_path_handler_cmd;

  selected_path_handler_cmd.data = "HardPathHandler";

  rclcpp::QoS qos = nav2::qos::LatchedPublisherQoS();

  auto path_handler_selector_pub =
    node_->create_publisher<std_msgs::msg::String>("path_handler_selector_custom_topic_name", qos);
  path_handler_selector_pub->on_activate();

  // publish a few updates of the selected_path_handler
  auto start = node_->now();
  while ((node_->now() - start).seconds() < 0.5) {
    tree_->rootNode()->executeTick();
    path_handler_selector_pub->publish(selected_path_handler_cmd);

    executor_->spin_some();
  }

  // check path_handler updated
  EXPECT_TRUE(config_->blackboard->get("selected_path_handler", selected_path_handler_result));
  EXPECT_EQ("HardPathHandler", selected_path_handler_result);
}

TEST_F(PathHandlerSelectorTestFixture, test_default_topic)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <PathHandlerSelector selected_path_handler="{selected_path_handler}" default_path_handler="GridBased"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // check default value
  std::string selected_path_handler_result;
  EXPECT_TRUE(config_->blackboard->get("selected_path_handler", selected_path_handler_result));

  EXPECT_EQ(selected_path_handler_result, "GridBased");

  std_msgs::msg::String selected_path_handler_cmd;

  selected_path_handler_cmd.data = "RRT";

  rclcpp::QoS qos = nav2::qos::LatchedPublisherQoS();

  auto path_handler_selector_pub =
    node_->create_publisher<std_msgs::msg::String>("path_handler_selector", qos);
  path_handler_selector_pub->on_activate();

  // publish a few updates of the selected_path_handler
  auto start = node_->now();
  while ((node_->now() - start).seconds() < 0.5) {
    tree_->rootNode()->executeTick();
    path_handler_selector_pub->publish(selected_path_handler_cmd);

    executor_->spin_some();
  }

  // check path_handler updated
  EXPECT_TRUE(config_->blackboard->get("selected_path_handler", selected_path_handler_result));
  EXPECT_EQ("RRT", selected_path_handler_result);
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
