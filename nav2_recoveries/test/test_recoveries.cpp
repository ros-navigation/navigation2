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
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <chrono>
#include <iostream>
#include <future>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_recoveries/recovery.hpp"
#include "nav2_msgs/action/dummy_recovery.hpp"

using nav2_recoveries::Recovery;
using nav2_recoveries::Status;
using RecoveryAction = nav2_msgs::action::DummyRecovery;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<RecoveryAction>;

using namespace std::chrono_literals;

// A recovery for testing the base class

class DummyRecovery : public Recovery<RecoveryAction>
{
public:
  DummyRecovery()
  : Recovery<RecoveryAction>(),
    initialized_(false) {}

  ~DummyRecovery() {}

  Status onRun(const std::shared_ptr<const RecoveryAction::Goal> goal) override
  {
    // A normal recovery would catch the command and initialize
    initialized_ = false;
    command_ = goal->command.data;
    start_time_ = std::chrono::system_clock::now();

    // onRun method can have various possible outcomes (success, failure, cancelled)
    // The output is defined by the tester class on the command string.
    if (command_ == "Testing success" || command_ == "Testing failure on run") {
      initialized_ = true;
      return Status::SUCCEEDED;
    }

    return Status::FAILED;
  }

  Status onCycleUpdate() override
  {
    // A normal recovery would set the robot in motion in the first call
    // and check for robot states on subsequent calls to check if the movement
    // was completed.

    if (command_ != "Testing success" || !initialized_) {
      return Status::FAILED;
    }

    // For testing, pretend the robot takes some fixed
    // amount of time to complete the motion.
    auto current_time = std::chrono::system_clock::now();
    auto motion_duration = 1s;

    if (current_time - start_time_ >= motion_duration) {
      // Movement was completed
      return Status::SUCCEEDED;
    }

    return Status::RUNNING;
  }

private:
  bool initialized_;
  std::string command_;
  std::chrono::system_clock::time_point start_time_;
};

// Define a test class to hold the context for the tests

class RecoveryTest : public ::testing::Test
{
protected:
  RecoveryTest() {SetUp();}
  ~RecoveryTest() {}

  void SetUp()
  {
    node_lifecycle_ =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "LifecycleRecoveryTestNode", rclcpp::NodeOptions());
    node_lifecycle_->declare_parameter(
      "costmap_topic",
      rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
    node_lifecycle_->declare_parameter(
      "footprint_topic",
      rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_lifecycle_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_lifecycle_->get_node_base_interface(),
      node_lifecycle_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string costmap_topic, footprint_topic;
    node_lifecycle_->get_parameter("costmap_topic", costmap_topic);
    node_lifecycle_->get_parameter("footprint_topic", footprint_topic);
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_ =
      std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
      node_lifecycle_, costmap_topic);
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_ =
      std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
      node_lifecycle_, footprint_topic, 1.0);
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_ =
      std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
      *costmap_sub_, *footprint_sub_, *tf_buffer_,
      node_lifecycle_->get_name(), "odom");

    recovery_ = std::make_shared<DummyRecovery>();
    recovery_->configure(node_lifecycle_, "Recovery", tf_buffer_, collision_checker_);
    recovery_->activate();

    client_ = rclcpp_action::create_client<RecoveryAction>(
      node_lifecycle_->get_node_base_interface(),
      node_lifecycle_->get_node_graph_interface(),
      node_lifecycle_->get_node_logging_interface(),
      node_lifecycle_->get_node_waitables_interface(), "Recovery");
    std::cout << "Setup complete." << std::endl;
  }

  void TearDown() override {}

  bool sendCommand(const std::string & command)
  {
    if (!client_->wait_for_action_server(4s)) {
      std::cout << "Server not up" << std::endl;
      return false;
    }

    auto goal = RecoveryAction::Goal();
    goal.command.data = command;
    auto future_goal = client_->async_send_goal(goal);

    if (rclcpp::spin_until_future_complete(node_lifecycle_, future_goal) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cout << "failed sending goal" << std::endl;
      // failed sending the goal
      return false;
    }

    goal_handle_ = future_goal.get();

    if (!goal_handle_) {
      std::cout << "goal was rejected" << std::endl;
      // goal was rejected by the action server
      return false;
    }

    return true;
  }

  Status getOutcome()
  {
    if (getResult().code == rclcpp_action::ResultCode::SUCCEEDED) {
      return Status::SUCCEEDED;
    }

    return Status::FAILED;
  }

  ClientGoalHandle::WrappedResult getResult()
  {
    std::cout << "Getting async result..." << std::endl;
    auto future_result = client_->async_get_result(goal_handle_);
    std::cout << "Waiting on future..." << std::endl;
    rclcpp::spin_until_future_complete(node_lifecycle_, future_result);
    std::cout << "future received!" << std::endl;
    return future_result.get();
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_lifecycle_;
  std::shared_ptr<DummyRecovery> recovery_;
  std::shared_ptr<rclcpp_action::Client<RecoveryAction>> client_;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<RecoveryAction>> goal_handle_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

// Define the tests

TEST_F(RecoveryTest, testingSuccess)
{
  ASSERT_TRUE(sendCommand("Testing success"));
  EXPECT_EQ(getOutcome(), Status::SUCCEEDED);
  SUCCEED();
}

TEST_F(RecoveryTest, testingFailureOnRun)
{
  ASSERT_TRUE(sendCommand("Testing failure on run"));
  EXPECT_EQ(getOutcome(), Status::FAILED);
  SUCCEED();
}

TEST_F(RecoveryTest, testingFailureOnInit)
{
  ASSERT_TRUE(sendCommand("Testing failure on init"));
  EXPECT_EQ(getOutcome(), Status::FAILED);
  SUCCEED();
}

TEST_F(RecoveryTest, testingSequentialFailures)
{
  ASSERT_TRUE(sendCommand("Testing failure on run"));
  EXPECT_EQ(getOutcome(), Status::FAILED);
  SUCCEED();
}

TEST_F(RecoveryTest, testingTotalElapsedTimeIsGratherThanZeroIfStarted)
{
  ASSERT_TRUE(sendCommand("Testing success"));
  EXPECT_GT(getResult().result->total_elapsed_time.sec, 0.0);
  SUCCEED();
}

TEST_F(RecoveryTest, testingTotalElapsedTimeIsZeroIfFailureOnInit)
{
  ASSERT_TRUE(sendCommand("Testing failure on init"));
  EXPECT_EQ(getResult().result->total_elapsed_time.sec, 0.0);
  SUCCEED();
}

TEST_F(RecoveryTest, testingTotalElapsedTimeIsZeroIfFailureOnRun)
{
  ASSERT_TRUE(sendCommand("Testing failure on run"));
  EXPECT_EQ(getResult().result->total_elapsed_time.sec, 0.0);
  SUCCEED();
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
