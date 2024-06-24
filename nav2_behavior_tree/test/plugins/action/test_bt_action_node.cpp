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
#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_action_node.hpp"

#include "test_msgs/action/fibonacci.hpp"

using namespace std::chrono_literals; // NOLINT
using namespace std::placeholders;  // NOLINT

class FibonacciActionServer : public rclcpp::Node
{
public:
  FibonacciActionServer()
  : rclcpp::Node("fibonacci_node", rclcpp::NodeOptions()),
    sleep_duration_(0ms)
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

  void setHandleGoalSleepDuration(std::chrono::milliseconds sleep_duration)
  {
    sleep_duration_ = sleep_duration;
  }

  void setServerLoopRate(std::chrono::nanoseconds server_loop_rate)
  {
    server_loop_rate_ = server_loop_rate;
  }

protected:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const test_msgs::action::Fibonacci::Goal>)
  {
    RCLCPP_INFO(this->get_logger(), "Goal is received..");
    if (sleep_duration_ > 0ms) {
      std::this_thread::sleep_for(sleep_duration_);
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>> handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), handle}.detach();
  }

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>> handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    if (handle) {
      const auto goal = handle->get_goal();
      auto result = std::make_shared<test_msgs::action::Fibonacci::Result>();

      if (goal->order < 0) {
        handle->abort(result);
        return;
      }

      auto & sequence = result->sequence;
      sequence.push_back(0);
      sequence.push_back(1);

      rclcpp::Rate rate(server_loop_rate_);
      for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
        if (handle->is_canceling()) {
          RCLCPP_INFO(this->get_logger(), "Goal is canceling.");
          handle->canceled(result);
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Goal is feedbacking.");
        sequence.push_back(sequence[i] + sequence[i - 1]);
        rate.sleep();
      }

      handle->succeed(result);
    }
  }

protected:
  rclcpp_action::Server<test_msgs::action::Fibonacci>::SharedPtr action_server_;
  std::chrono::milliseconds sleep_duration_;
  std::chrono::nanoseconds server_loop_rate_;
};

class FibonacciAction : public nav2_behavior_tree::BtActionNode<test_msgs::action::Fibonacci>
{
public:
  FibonacciAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<test_msgs::action::Fibonacci>(xml_tag_name, "fibonacci", conf)
  {}

  void on_tick() override
  {
    getInput("order", goal_.order);
  }

  BT::NodeStatus on_success() override
  {
    config().blackboard->set<std::vector<int>>("sequence", result_.result->sequence);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus on_cancelled() override
  {
    config().blackboard->set<std::vector<int>>("sequence", result_.result->sequence);
    config().blackboard->set<bool>("on_cancelled_triggered", true);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<int>("order", "Fibonacci order")});
  }
};

class BTActionNodeTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("bt_action_node_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 20ms);
    config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
    config_->blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 1000ms);
    config_->blackboard->set<bool>("initial_pose_received", false);
    config_->blackboard->set<bool>("on_cancelled_triggered", false);

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

  void SetUp() override
  {
    // initialize action server and spin on new thread
    action_server_ = std::make_shared<FibonacciActionServer>();
    server_thread_ = std::make_shared<std::thread>(
      []() {
        while (rclcpp::ok() && BTActionNodeTestFixture::action_server_ != nullptr) {
          rclcpp::spin_some(BTActionNodeTestFixture::action_server_);
          std::this_thread::sleep_for(100ns);
        }
      });
  }

  void TearDown() override
  {
    action_server_.reset();
    tree_.reset();
    server_thread_->join();
    server_thread_.reset();
  }

  static std::shared_ptr<FibonacciActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
  static std::shared_ptr<std::thread> server_thread_;
};

rclcpp::Node::SharedPtr BTActionNodeTestFixture::node_ = nullptr;
std::shared_ptr<FibonacciActionServer> BTActionNodeTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * BTActionNodeTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> BTActionNodeTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> BTActionNodeTestFixture::tree_ = nullptr;
std::shared_ptr<std::thread> BTActionNodeTestFixture::server_thread_ = nullptr;

TEST_F(BTActionNodeTestFixture, test_server_timeout_success)
{
  // create tree
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Fibonacci order="5" />
        </BehaviorTree>
      </root>)";

  // the server timeout is larger than the goal handling duration
  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 20ms);
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // setting a small action server goal handling duration
  action_server_->setHandleGoalSleepDuration(2ms);
  action_server_->setServerLoopRate(10ns);

  // to keep track of the number of ticks it took to reach a terminal result
  int ticks = 0;

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // BT loop execution rate
  rclcpp::WallRate loopRate(10ms);

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }

  // get calculated fibonacci sequence from blackboard
  auto sequence = config_->blackboard->get<std::vector<int>>("sequence");

  // expected fibonacci sequence for order 5
  std::vector<int> expected = {0, 1, 1, 2, 3, 5};

  // since the server timeout was larger than the action server goal handling duration
  // the BT should have succeeded
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);

  // checking the output fibonacci sequence
  EXPECT_EQ(sequence.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(sequence[i], expected[i]);
  }

  // start a new execution cycle with the previous BT to ensure previous state doesn't leak into
  // the new cycle

  // halt BT for a new execution cycle,
  // get if the on_cancelled is triggered from blackboard and assert
  // that the on_cancelled triggers after halting node
  RCLCPP_INFO(node_->get_logger(), "Tree is halting.");
  tree_->haltTree();
  bool on_cancelled_triggered = config_->blackboard->get<bool>("on_cancelled_triggered");
  EXPECT_EQ(on_cancelled_triggered, false);

  // setting a large action server goal handling duration
  action_server_->setHandleGoalSleepDuration(100ms);
  action_server_->setServerLoopRate(10ns);

  // reset state variables
  ticks = 0;
  result = BT::NodeStatus::RUNNING;
  config_->blackboard->set<bool>("on_cancelled_triggered", false);

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }

  // since the server timeout was smaller than the action server goal handling duration
  // the BT should have failed
  EXPECT_EQ(result, BT::NodeStatus::FAILURE);

  // since the server timeout is 20ms and bt loop duration is 10ms, number of ticks should
  // be at most 2, but it can be 1 too, because the tickOnce may execute two ticks.
  EXPECT_LE(ticks, 3);
  EXPECT_GE(ticks, 1);
}

TEST_F(BTActionNodeTestFixture, test_server_timeout_failure)
{
  // create tree
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Fibonacci order="2" />
        </BehaviorTree>
      </root>)";

  // setting a server timeout smaller than the time the action server will take to accept the goal
  // to simulate a server timeout scenario
  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 90ms);
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // the action server will take 100ms before accepting the goal
  action_server_->setHandleGoalSleepDuration(100ms);
  action_server_->setServerLoopRate(10ns);

  // to keep track of the number of ticks it took to reach a terminal result
  int ticks = 0;

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // BT loop execution rate
  rclcpp::WallRate loopRate(10ms);

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }

  // since the server timeout was smaller than the action server goal handling duration
  // the BT should have failed
  EXPECT_EQ(result, BT::NodeStatus::FAILURE);

  // since the server timeout is 90ms and bt loop duration is 10ms, number of ticks should be 9
  EXPECT_EQ(ticks, 10);

  // start a new execution cycle with the previous BT to ensure previous state doesn't leak into
  // the new cycle

  // halt BT for a new execution cycle
  // get if the on_cancel is triggered from blackboard and assert
  // that the on_cancelled never can trigger after halting node
  RCLCPP_INFO(node_->get_logger(), "Tree is halting.");
  tree_->haltTree();
  bool on_cancelled_triggered = config_->blackboard->get<bool>("on_cancelled_triggered");
  EXPECT_EQ(on_cancelled_triggered, false);

  // setting a small action server goal handling duration
  action_server_->setHandleGoalSleepDuration(25ms);
  action_server_->setServerLoopRate(10ns);

  // reset state variables
  ticks = 0;
  result = BT::NodeStatus::RUNNING;
  config_->blackboard->set<bool>("on_cancelled_triggered", false);

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }

  // since the server timeout was smaller than the action server goal handling duration
  // the BT should have failed
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BTActionNodeTestFixture, test_server_cancel)
{
  // create tree
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Fibonacci order="1000000" />
        </BehaviorTree>
      </root>)";

  // setting a server timeout smaller than the time the action server will take to accept the goal
  // to simulate a server timeout scenario
  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 100ms);
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // the action server will take 2ms before accepting the goal
  // and the feedback period of the action server will be 50ms
  action_server_->setHandleGoalSleepDuration(2ms);
  action_server_->setServerLoopRate(50ms);

  // to keep track of the number of ticks it took to reach expected tick count
  int ticks = 0;

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // BT loop execution rate
  rclcpp::WallRate loopRate(100ms);

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING && ticks < 5) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }

  // halt BT for testing if the action node cancels the goal correctly
  RCLCPP_INFO(node_->get_logger(), "Tree is halting.");
  tree_->haltTree();

  // get if the on_cancel is triggered from blackboard and assert
  // that the on_cancel is triggered after halting node
  bool on_cancelled_triggered = config_->blackboard->get<bool>("on_cancelled_triggered");
  EXPECT_EQ(on_cancelled_triggered, true);

  // ticks variable must be 5 because execution time of the action server
  // is at least 1000000 x 50 ms
  EXPECT_EQ(ticks, 5);

  // send new goal to the action server for a new execution cycle

  // the action server will take 2ms before accepting the goal
  // and the feedback period of the action server will be 1000ms
  action_server_->setHandleGoalSleepDuration(2ms);
  action_server_->setServerLoopRate(50ms);

  // reset state variable
  ticks = 0;
  config_->blackboard->set<bool>("on_cancelled_triggered", false);
  result = BT::NodeStatus::RUNNING;

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING && ticks < 7) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }

  // halt BT for testing if the action node cancels the goal correctly
  RCLCPP_INFO(node_->get_logger(), "Tree is halting.");
  tree_->haltTree();

  // get if the on_cancel is triggered from blackboard and assert
  // that the on_cancel is triggered after halting node
  on_cancelled_triggered = config_->blackboard->get<bool>("on_cancelled_triggered");
  EXPECT_EQ(on_cancelled_triggered, true);

  // ticks variable must be 7 because execution time of the action server
  // is at least 1000000 x 50 ms
  EXPECT_EQ(ticks, 7);
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
