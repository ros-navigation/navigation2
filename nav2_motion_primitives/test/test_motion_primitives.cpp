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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_motion_primitives/motion_primitive.hpp"
#include "nav2_msgs/action/dummy_primitive.hpp"

using nav2_motion_primitives::MotionPrimitive;
using nav2_motion_primitives::Status;
using MotionPrimitiveAction = nav2_msgs::action::DummyPrimitive;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<MotionPrimitiveAction>;

using namespace std::chrono_literals;

// A motion primitive for testing the base class

class DummyPrimitive : public MotionPrimitive<MotionPrimitiveAction>
{
public:
  explicit DummyPrimitive(rclcpp::Node::SharedPtr & node)
  : MotionPrimitive<MotionPrimitiveAction>(node, "MotionPrimitive"),
    initialized_(false) {}

  ~DummyPrimitive() {}

  Status onRun(const std::shared_ptr<const MotionPrimitiveAction::Goal> goal) override
  {
    // A normal primitive would catch the command and initialize
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
    // A normal primitive would set the robot in motion in the first call
    // and check for robot states on subsequent calls to check if the movement
    // was completed.

    if (command_ != "Testing success" || !initialized_) {
      return Status::FAILED;
    }

    // Fake getting the robot state, calculate and send control output
    std::this_thread::sleep_for(2ms);

    // For testing, pretend the robot takes some fixed
    // amount of time to complete the motion.
    auto current_time = std::chrono::system_clock::now();
    auto motion_duration = 5s;

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

class MotionPrimitivesTest : public ::testing::Test
{
protected:
  MotionPrimitivesTest() {}
  ~MotionPrimitivesTest() {}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("MotionPrimitivesTestNode");

    primitive_ = std::make_unique<DummyPrimitive>(node_);

    client_ = rclcpp_action::create_client<MotionPrimitiveAction>(node_, "MotionPrimitive");
  }

  void TearDown() override {}

  bool sendCommand(const std::string & command)
  {
    if (!client_->wait_for_action_server(4s)) {
      return false;
    }

    auto future_goal = getGoal(command);

    if (rclcpp::spin_until_future_complete(node_, future_goal) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      // failed sending the goal
      return false;
    }

    goal_handle_ = future_goal.get();

    if (!goal_handle_) {
      // goal was rejected by the action server
      return false;
    }

    return true;
  }

  std::shared_future<ClientGoalHandle::SharedPtr> getGoal(const std::string & command)
  {
    auto goal = MotionPrimitiveAction::Goal();
    goal.command.data = command;

    auto goal_options = rclcpp_action::Client<MotionPrimitiveAction>::SendGoalOptions();
    goal_options.result_callback = [](auto) {};

    return client_->async_send_goal(goal, goal_options);
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
    auto future_result = goal_handle_->async_result();
    rclcpp::executor::FutureReturnCode frc;

    do {
      frc = rclcpp::spin_until_future_complete(node_, future_result);
    } while (frc != rclcpp::executor::FutureReturnCode::SUCCESS);

    return future_result.get();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<DummyPrimitive> primitive_;
  std::shared_ptr<rclcpp_action::Client<MotionPrimitiveAction>> client_;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<MotionPrimitiveAction>> goal_handle_;
};

// Define the tests

TEST_F(MotionPrimitivesTest, testingSuccess)
{
  ASSERT_TRUE(sendCommand("Testing success"));
  EXPECT_EQ(getOutcome(), Status::SUCCEEDED);
}

TEST_F(MotionPrimitivesTest, testingFailureOnRun)
{
  ASSERT_TRUE(sendCommand("Testing failure on run"));
  EXPECT_EQ(getOutcome(), Status::FAILED);
}

TEST_F(MotionPrimitivesTest, testingFailureOnInit)
{
  ASSERT_TRUE(sendCommand("Testing failure on init"));
  EXPECT_EQ(getOutcome(), Status::FAILED);
}

TEST_F(MotionPrimitivesTest, testingSequentialFailures)
{
  ASSERT_TRUE(sendCommand("Testing failure on init"));
  EXPECT_EQ(getOutcome(), Status::FAILED);

  ASSERT_TRUE(sendCommand("Testing failure on run"));
  EXPECT_EQ(getOutcome(), Status::FAILED);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(0, nullptr);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
