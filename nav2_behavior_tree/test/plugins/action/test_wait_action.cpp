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
#include <condition_variable>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/wait_action.hpp"

class WaitActionServer : public TestActionServer<nav2_msgs::action::Wait>
{
public:
  WaitActionServer()
  : TestActionServer("wait")
  {}

  void setGoalResponse(rclcpp_action::GoalResponse goal_response)
  {
    goal_response_ = goal_response;
  }

  void blockGoalResponse()
  {
    std::lock_guard<std::mutex> lock(goal_response_mutex_);
    goal_response_blocked_ = true;
    goal_request_received_ = false;
  }

  void releaseGoalResponse()
  {
    {
      std::lock_guard<std::mutex> lock(goal_response_mutex_);
      goal_response_blocked_ = false;
    }
    goal_response_cv_.notify_all();
  }

  void waitForGoalRequest()
  {
    std::unique_lock<std::mutex> lock(goal_response_mutex_);
    goal_response_cv_.wait(lock, [this]() {return goal_request_received_;});
  }

protected:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & goal_id,
    std::shared_ptr<const nav2_msgs::action::Wait::Goal> goal) override
  {
    {
      std::lock_guard<std::mutex> lock(goal_response_mutex_);
      goal_request_received_ = true;
    }
    goal_response_cv_.notify_all();

    {
      std::unique_lock<std::mutex> lock(goal_response_mutex_);
      goal_response_cv_.wait(lock, [this]() {return !goal_response_blocked_;});
    }

    if (goal_response_ == rclcpp_action::GoalResponse::REJECT) {
      return goal_response_;
    }

    return TestActionServer<nav2_msgs::action::Wait>::handle_goal(goal_id, goal);
  }

  void execute(
    const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::Wait>>
    goal_handle)
  override
  {
    auto result = std::make_shared<nav2_msgs::action::Wait::Result>();
    goal_handle->succeed(result);
  }

  rclcpp_action::GoalResponse goal_response_{rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE};
  std::condition_variable goal_response_cv_;
  std::mutex goal_response_mutex_;
  bool goal_response_blocked_{false};
  bool goal_request_received_{false};
};

class TestableWaitAction : public nav2_behavior_tree::WaitAction
{
public:
  using WaitAction::WaitAction;

  void cancelExecutor()
  {
    callback_group_executor_.cancel();
  }
};

class WaitActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("wait_action_test_fixture");
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
    config_->blackboard->set("number_recoveries", 0);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::WaitAction>(
          name, "wait", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::WaitAction>("Wait", builder);

    BT::NodeBuilder testable_builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<TestableWaitAction>(name, "wait", config);
      };

    factory_->registerBuilder<TestableWaitAction>("TestableWait", testable_builder);
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
    config_->blackboard->set("number_recoveries", 0);
    action_server_->setGoalResponse(rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<WaitActionServer> action_server_;

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

nav2::LifecycleNode::SharedPtr WaitActionTestFixture::node_ = nullptr;
std::shared_ptr<WaitActionServer> WaitActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * WaitActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> WaitActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> WaitActionTestFixture::tree_ = nullptr;

TEST_F(WaitActionTestFixture, test_ports)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Wait />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<double>("wait_duration"), 1.0);

  xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Wait wait_duration="10.0" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<double>("wait_duration"), 10.0);
}

TEST_F(WaitActionTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Wait wait_duration="-5.0"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 0);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 1);
  EXPECT_EQ(rclcpp::Duration(action_server_->getCurrentGoal()->time).seconds(), 5.0);
}

TEST_F(WaitActionTestFixture, test_goal_rejected_error_code)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Wait error_code_id="{wait_error_code}" error_msg="{wait_error_msg}" />
        </BehaviorTree>
      </root>)";

  action_server_->setGoalResponse(rclcpp_action::GoalResponse::REJECT);
  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(
    config_->blackboard->get<uint16_t>("wait_error_code"),
    nav2_msgs::action::Wait::Result::GOAL_REJECTED);
  EXPECT_EQ(
    config_->blackboard->get<std::string>("wait_error_msg"),
    "Goal was rejected by the action server.");
}

TEST_F(WaitActionTestFixture, test_send_goal_failure_error_code)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <TestableWait error_code_id="{wait_error_code}" error_msg="{wait_error_msg}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  auto * action = dynamic_cast<TestableWaitAction *>(tree_->rootNode());
  ASSERT_NE(action, nullptr);

  action_server_->blockGoalResponse();
  std::thread cancel_thread([action]() {
      WaitActionTestFixture::action_server_->waitForGoalRequest();
      action->cancelExecutor();
    });

  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::FAILURE);
  action_server_->releaseGoalResponse();
  cancel_thread.join();
  EXPECT_EQ(
    config_->blackboard->get<uint16_t>("wait_error_code"),
    nav2_msgs::action::Wait::Result::SEND_GOAL_FAILURE);
  EXPECT_EQ(
    config_->blackboard->get<std::string>("wait_error_msg"),
    "Failed to send goal to the action server.");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and spin on new thread
  WaitActionTestFixture::action_server_ = std::make_shared<WaitActionServer>();
  std::thread server_thread([]() {
      rclcpp::spin(WaitActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
