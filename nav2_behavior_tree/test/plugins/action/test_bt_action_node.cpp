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
#include <atomic>
#include <cstdint>
#include <memory>
#include <set>
#include <vector>
#include <string>
#include <chrono>
#include <future>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_behavior_tree/utils/loop_rate.hpp"

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

  void setGoalResponse(rclcpp_action::GoalResponse goal_response)
  {
    goal_response_ = goal_response;
  }

  unsigned int getAcceptedGoalCount() const
  {
    return accepted_goal_count_.load();
  }

  unsigned int getCancelRequestCount() const
  {
    return cancel_request_count_.load();
  }

  // Withhold the goal acknowledgment until releaseGoalAck() (keeps goal_handle_ null).
  void gateGoalAck()
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    ack_gate_promise_ = std::make_shared<std::promise<void>>();
    ack_gate_future_ = ack_gate_promise_->get_future().share();
  }

  // Release a previously installed acknowledgment gate.
  void releaseGoalAck()
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    if (ack_gate_promise_) {
      ack_gate_promise_->set_value();
      ack_gate_promise_.reset();
    }
    ack_gate_future_ = std::shared_future<void>();
  }

protected:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const test_msgs::action::Fibonacci::Goal>)
  {
    RCLCPP_INFO(this->get_logger(), "Goal is received..");
    // Optional ACK gate: copy the future under the lock, then wait without
    // holding it so releaseGoalAck() cannot deadlock.
    std::shared_future<void> gate;
    {
      std::lock_guard<std::mutex> lock(gate_mutex_);
      gate = ack_gate_future_;
    }
    if (gate.valid()) {
      gate.wait();
    }
    if (sleep_duration_ > 0ms) {
      std::this_thread::sleep_for(sleep_duration_);
    }
    return goal_response_;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>>)
  {
    cancel_request_count_++;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>> handle)
  {
    accepted_goal_count_++;
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
  rclcpp_action::GoalResponse goal_response_{rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE};
  std::atomic_uint accepted_goal_count_{0};
  std::atomic_uint cancel_request_count_{0};
  std::mutex gate_mutex_;
  std::shared_ptr<std::promise<void>> ack_gate_promise_;
  std::shared_future<void> ack_gate_future_;
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

  void on_wait_for_result(
    std::shared_ptr<const test_msgs::action::Fibonacci::Feedback>) override
  {
    if (config().blackboard->get<bool>("goal_updated")) {
      goal_updated_ = true;
      config().blackboard->set("goal_updated", false);
    }
  }

  BT::NodeStatus on_success() override
  {
    config().blackboard->set("sequence", result_.result->sequence);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus on_cancelled() override
  {
    if (result_.result) {
      config().blackboard->set("sequence", result_.result->sequence);
    }
    config().blackboard->set("on_cancelled_triggered", true);
    return BT::NodeStatus::SUCCESS;
  }

  void on_goal_rejected() override
  {
    setOutput("error_code_id", GOAL_REJECTED_ERROR_CODE);
    config().blackboard->set("on_goal_rejected_triggered", true);
  }

  void on_send_goal_failure() override
  {
    setOutput("error_code_id", SEND_GOAL_FAILURE_ERROR_CODE);
    config().blackboard->set("on_send_goal_failure_triggered", true);
  }

  static constexpr uint16_t GOAL_REJECTED_ERROR_CODE = 1;
  static constexpr uint16_t SEND_GOAL_FAILURE_ERROR_CODE = 2;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
    {
      BT::InputPort<int>("order", "Fibonacci order"),
      });
  }
};

class BTActionNodeTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("bt_action_node_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 20ms);
    config_->blackboard->set<std::chrono::milliseconds>("cancel_timeout", 50ms);
    config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
    config_->blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 1000ms);
    config_->blackboard->set("initial_pose_received", false);
    config_->blackboard->set("on_cancelled_triggered", false);
    config_->blackboard->set("on_goal_rejected_triggered", false);
    config_->blackboard->set("on_send_goal_failure_triggered", false);
    config_->blackboard->set("goal_updated", false);

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
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(action_server_);
        while (rclcpp::ok() && BTActionNodeTestFixture::action_server_ != nullptr) {
          executor.spin_some();
          std::this_thread::sleep_for(100ns);
        }
      });
  }

  void TearDown() override
  {
    // Sleep for some time to avoid race condition
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    action_server_.reset();
    tree_.reset();
    server_thread_->join();
    server_thread_.reset();
  }

  static std::shared_ptr<FibonacciActionServer> action_server_;

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
  static std::shared_ptr<std::thread> server_thread_;
};

nav2::LifecycleNode::SharedPtr BTActionNodeTestFixture::node_ = nullptr;
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
      <root BTCPP_format="4">
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
  nav2_behavior_tree::LoopRate loopRate(
    10ms, tree_.get(), std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME));

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickOnce();
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
  config_->blackboard->set("on_cancelled_triggered", false);

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickOnce();
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
      <root BTCPP_format="4">
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
    result = tree_->tickOnce();
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
  config_->blackboard->set("on_cancelled_triggered", false);

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickOnce();
    ticks++;
    loopRate.sleep();
  }

  // since the server timeout was smaller than the action server goal handling duration
  // the BT should have failed
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST_F(BTActionNodeTestFixture, test_updated_goal_timeout_cancels_all_goals)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Fibonacci order="1000000" />
        </BehaviorTree>
      </root>)";

  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 20ms);
  config_->blackboard->set<std::chrono::milliseconds>("cancel_timeout", 150ms);
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
  config_->blackboard->set("goal_updated", false);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  action_server_->setHandleGoalSleepDuration(1ms);
  action_server_->setServerLoopRate(10ms);

  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::RUNNING);

  const auto first_goal_deadline = std::chrono::steady_clock::now() + 100ms;
  while (action_server_->getAcceptedGoalCount() < 1u &&
    std::chrono::steady_clock::now() < first_goal_deadline)
  {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(action_server_->getAcceptedGoalCount(), 1u);

  // Delay the replacement goal response past the BT action client's timeout.
  action_server_->setHandleGoalSleepDuration(100ms);
  config_->blackboard->set("goal_updated", true);

  BT::NodeStatus result = BT::NodeStatus::RUNNING;
  rclcpp::WallRate loop_rate(10ms);
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickOnce();
    loop_rate.sleep();
  }

  EXPECT_EQ(result, BT::NodeStatus::FAILURE);

  const auto cancel_deadline = std::chrono::steady_clock::now() + 200ms;
  while (action_server_->getCancelRequestCount() < 2u &&
    std::chrono::steady_clock::now() < cancel_deadline)
  {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_EQ(action_server_->getAcceptedGoalCount(), 2u);
  EXPECT_EQ(action_server_->getCancelRequestCount(), 2u);
}

TEST_F(BTActionNodeTestFixture, test_updated_goal_immediate_timeout_cancels_all_goals)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Fibonacci order="1000000" />
        </BehaviorTree>
      </root>)";

  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 20ms);
  config_->blackboard->set<std::chrono::milliseconds>("cancel_timeout", 150ms);
  // Make max_timeout_ larger than server_timeout_ so the updated goal times out in the
  // immediate goal-response check.
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 100ms);
  config_->blackboard->set("goal_updated", false);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  action_server_->setHandleGoalSleepDuration(1ms);
  action_server_->setServerLoopRate(10ms);

  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::RUNNING);

  const auto first_goal_deadline = std::chrono::steady_clock::now() + 100ms;
  while (action_server_->getAcceptedGoalCount() < 1u &&
    std::chrono::steady_clock::now() < first_goal_deadline)
  {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(action_server_->getAcceptedGoalCount(), 1u);

  action_server_->setHandleGoalSleepDuration(100ms);
  config_->blackboard->set("goal_updated", true);

  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::FAILURE);

  const auto cancel_deadline = std::chrono::steady_clock::now() + 200ms;
  while (action_server_->getCancelRequestCount() < 2u &&
    std::chrono::steady_clock::now() < cancel_deadline)
  {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_EQ(action_server_->getAcceptedGoalCount(), 2u);
  EXPECT_EQ(action_server_->getCancelRequestCount(), 2u);
}

TEST_F(BTActionNodeTestFixture, test_goal_rejected)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Fibonacci order="2" error_code_id="{fibonacci_error_code}" />
        </BehaviorTree>
      </root>)";

  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 100ms);
  config_->blackboard->set("on_goal_rejected_triggered", false);
  action_server_->setGoalResponse(rclcpp_action::GoalResponse::REJECT);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(config_->blackboard->get<bool>("on_goal_rejected_triggered"));
  EXPECT_EQ(config_->blackboard->get<uint16_t>("fibonacci_error_code"),
    FibonacciAction::GOAL_REJECTED_ERROR_CODE);
}

TEST_F(BTActionNodeTestFixture, test_server_cancel)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
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
    result = tree_->tickOnce();
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
  config_->blackboard->set("on_cancelled_triggered", false);
  result = BT::NodeStatus::RUNNING;

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING && ticks < 7) {
    result = tree_->tickOnce();
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

TEST_F(BTActionNodeTestFixture, test_cancel_pending_goal_on_halt)
{
  // #6426: halting a node whose goal is still pending (goal_handle_ null) must
  // still cancel that goal once acknowledged, not leave it orphaned.
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Fibonacci order="1000000" />
        </BehaviorTree>
      </root>)";

  // Large server_timeout so a single tick yields RUNNING with the goal still
  // pending, and so halt() has ample budget to wait for the (gated) ack.
  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 2000ms);
  config_->blackboard->set<std::chrono::milliseconds>("cancel_timeout", 2000ms);
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  action_server_->setHandleGoalSleepDuration(0ms);
  action_server_->setServerLoopRate(10000000ns);  // 10ms feedback loop

  // Withhold the acknowledgment so the goal stays "sent but not acknowledged".
  action_server_->gateGoalAck();

  // One tick: the goal is dispatched; with the ack gated, the node cannot latch
  // the goal handle and must report RUNNING with the goal still pending.
  BT::NodeStatus status = tree_->tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(action_server_->getCancelRequestCount(), 0u);

  // Now let the server acknowledge, then halt the node. A correct implementation
  // resolves the pending goal handle and cancels the in-flight goal.
  action_server_->releaseGoalAck();
  tree_->haltTree();

  // The in-flight goal must have been cancelled exactly once (not orphaned).
  EXPECT_EQ(action_server_->getCancelRequestCount(), 1u)
    << "Halting a BtActionNode with a pending (unacknowledged) goal must cancel "
       "it once acknowledged; got no cancel request, i.e. the goal was orphaned.";
  EXPECT_EQ(action_server_->getAcceptedGoalCount(), 1u);
}

TEST_F(BTActionNodeTestFixture, test_halt_with_pending_rejected_goal)
{
  // #6426 companion: if the still-pending goal is rejected, halt() resolves the
  // handle, finds nothing to cancel, and completes without throwing.
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Fibonacci order="1000000" />
        </BehaviorTree>
      </root>)";

  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 2000ms);
  config_->blackboard->set<std::chrono::milliseconds>("cancel_timeout", 2000ms);
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  action_server_->setHandleGoalSleepDuration(0ms);
  action_server_->setServerLoopRate(10000000ns);
  action_server_->setGoalResponse(rclcpp_action::GoalResponse::REJECT);

  // Gate the response so the goal is dispatched but stays pending after the tick.
  action_server_->gateGoalAck();
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(action_server_->getCancelRequestCount(), 0u);

  // Release the response (server rejects); halting resolves the pending handle,
  // catches the rejection, and completes cleanly.
  action_server_->releaseGoalAck();
  EXPECT_NO_THROW(tree_->haltTree());

  // A rejected goal is never accepted, so no cancel request is sent.
  EXPECT_EQ(action_server_->getCancelRequestCount(), 0u);
  EXPECT_EQ(action_server_->getAcceptedGoalCount(), 0u);
}

TEST_F(BTActionNodeTestFixture, test_halt_with_pending_goal_timeout)
{
  // #6426 companion: if no acknowledgment arrives, halt()'s bounded wait returns
  // cleanly without sending a cancel and without hanging.
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Fibonacci order="1000000" />
        </BehaviorTree>
      </root>)";

  // Deliberately short, bounded server timeout so the pending-handle wait ends
  // deterministically when no acknowledgment arrives.
  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 200ms);
  config_->blackboard->set<std::chrono::milliseconds>("cancel_timeout", 200ms);
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  action_server_->setHandleGoalSleepDuration(0ms);
  // Reject on release so teardown never enters execute(); the gate stays closed
  // throughout halt(), so this still exercises the timeout path.
  action_server_->setGoalResponse(rclcpp_action::GoalResponse::REJECT);

  // Gate the acknowledgment and keep it withheld during halt: the goal stays
  // pending, so halt()'s wait loop exhausts its bounded budget and exits.
  action_server_->gateGoalAck();
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::RUNNING);

  EXPECT_NO_THROW(tree_->haltTree());
  EXPECT_EQ(action_server_->getCancelRequestCount(), 0u);

  // Release the gate so the server worker can unblock for teardown.
  action_server_->releaseGoalAck();
}

TEST_F(BTActionNodeTestFixture, test_halt_uses_remaining_timeout_budget)
{
  // #6426: halt() spends only the goal's REMAINING server_timeout_ budget, never
  // a fresh one. Once the budget is exhausted it must not spin, so a goal that is
  // acknowledged only afterwards is left un-latched and is not cancelled here.
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Fibonacci order="5" />
        </BehaviorTree>
      </root>)";

  const auto server_timeout = std::chrono::milliseconds(100);
  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", server_timeout);
  config_->blackboard->set<std::chrono::milliseconds>("cancel_timeout", 100ms);
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  action_server_->setHandleGoalSleepDuration(0ms);
  action_server_->setServerLoopRate(10000000ns);

  // Dispatch the goal and hold it pending.
  action_server_->gateGoalAck();
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::RUNNING);

  // Let the entire server_timeout_ budget elapse before the response is available.
  std::this_thread::sleep_for(server_timeout * 3);
  action_server_->releaseGoalAck();

  // The server does accept the goal, so the acknowledgment is genuinely available.
  for (int i = 0; i < 200 && action_server_->getAcceptedGoalCount() == 0u; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(action_server_->getAcceptedGoalCount(), 1u);

  // With no remaining budget halt() does not spin, does not latch the handle, and
  // issues no cancel. A fresh-timeout implementation would instead cancel it.
  tree_->haltTree();
  EXPECT_EQ(action_server_->getCancelRequestCount(), 0u);
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
