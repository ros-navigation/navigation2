// Copyright (c) 2022 Neobotix GmbH
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
#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_cancel_action_node.hpp"

#include "test_msgs/action/fibonacci.hpp"

using namespace std::chrono_literals; // NOLINT
using namespace std::placeholders;  // NOLINT

class FibonacciActionServer : public rclcpp::Node
{
public:
  FibonacciActionServer()
  : rclcpp::Node("fibonacci_node", rclcpp::NodeOptions())
  {
    this->action_server_ = rclcpp_action::create_server<test_msgs::action::Fibonacci>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  }

protected:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const test_msgs::action::Fibonacci::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>>)
  {
  }

protected:
  rclcpp_action::Server<test_msgs::action::Fibonacci>::SharedPtr action_server_;
  std::chrono::milliseconds sleep_duration_;
};

class FibonacciAction : public nav2_behavior_tree::BtCancelActionNode<test_msgs::action::Fibonacci>
{
public:
  FibonacciAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtCancelActionNode<test_msgs::action::Fibonacci>(xml_tag_name, "fibonacci",
      conf)
  {}
};

class BTCancelActionNodeTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("bt_cancel_action_node_test_fixture");
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

    client_ = rclcpp_action::create_client<test_msgs::action::Fibonacci>(
      node_, "fibonacci");

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<FibonacciAction>(name, config);
      };

    factory_->registerBuilder<FibonacciAction>("Fibonacci", builder);
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

  static std::shared_ptr<FibonacciActionServer> action_server_;
  static std::shared_ptr<rclcpp_action::Client<test_msgs::action::Fibonacci>> client_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr BTCancelActionNodeTestFixture::node_ = nullptr;
std::shared_ptr<FibonacciActionServer>
BTCancelActionNodeTestFixture::action_server_ = nullptr;
std::shared_ptr<rclcpp_action::Client<test_msgs::action::Fibonacci>>
BTCancelActionNodeTestFixture::client_ = nullptr;

BT::NodeConfiguration * BTCancelActionNodeTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> BTCancelActionNodeTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> BTCancelActionNodeTestFixture::tree_ = nullptr;

TEST_F(BTCancelActionNodeTestFixture, test_ports)
{
  // Creating a Fibonacci and treating it as a dummy node
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Fibonacci/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  auto send_goal_options = rclcpp_action::Client<test_msgs::action::Fibonacci>::SendGoalOptions();
  auto goal_msg = test_msgs::action::Fibonacci::Goal();

  client_->wait_for_action_server();
  // Sending a dummy goal
  client_->async_send_goal(goal_msg, send_goal_options);

  // Ticking the tree
  tree_->rootNode()->executeTick();

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and spin on new thread
  BTCancelActionNodeTestFixture::action_server_ = std::make_shared<FibonacciActionServer>();
  std::thread server_thread([]() {
      rclcpp::spin(BTCancelActionNodeTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
