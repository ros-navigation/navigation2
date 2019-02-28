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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_tasks/task_status.hpp"
#include "nav2_motion_primitives/motion_primitive.hpp"

using nav2_tasks::TaskStatus;
using namespace std::chrono_literals;

// A global object to initialize ROS

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};

RclCppFixture g_rclcppfixture;

// Let's create a motion primitive for testing the base class

using DummyPrimitiveCommand = std_msgs::msg::String;
using DummyPrimitiveResult = std_msgs::msg::String;

class DummyPrimitive : public MotionPrimitives<DummyPrimitiveCommand, DummyPrimitiveResult>
{
public:
  explicit DummyPrimitive(rclcpp::Node::SharedPtr & node)
  : MotionPrimitive<DummyPrimitiveCommand, DummyPrimitiveResult>(node),
    initialized_(false) {}

  ~DummyPrimitive() {}

  TaskStatus onRun(const DummyPrimitiveCommand::SharedPtr command) override
  {
    // A normal primitive would catch the command and initialize
    initialized_ = false;
    command_ = command;
    start_time_ = std::chrono::system_clock::now();

    // Method can have various possible outcomes (success, failure, cancelled)
    // To generate the possible outcomes we use the command string.

    if (command->data == "Testing failure") {
      // Means initialization failed or command is invalid
      return TaskStatus::FAILED;
    } else if (command->data == "Testing success") {
      // Means initialization succeeded
      initialzed_ = true;
      return TaskStatus::SUCCEEDED;
    } else {
      return TaskStatus::FAILED;
    }
  }

  TaskStatus onCycleUpdate(DummyPrimitiveResult & result) override
  {
    // A normal primitive would set the robot in motion in the first call
    // and check for robot states on subsequent calls to check if the movement
    // was completed

    if (command->data != "Testing success" || !initialized_) {
      return TaskStatus::FAILED;
    }

    auto current_time = std::chrono::system_clock::now();
    auto motion_duration = 5s;

    if (current_time - start_time_ >= motion_duration) {
      // Movement was completed
      return TaskStatus::SUCCEEDED;
    }

    // Some computation
    std::this_thread::sleep_for(5ms);

    return TaskStatus::RUNNING;
  }

private:
  bool initialized_;
  std::shared_ptr<DummyPrimitiveCommand> command_;
  std::chrono::system_clock::time_point start_time_;
}

// Define a test class to hold the context for the tests

class MotionPrimitivesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dummy_node_ = std::make_shared<rclcpp::Node>("DummyNode");

    // The motion primitives are not themselves a node
    primitive_ = std::make_unique<DummyPrimitive>(dummy_node_);
  }

  void TearDown() override {}

  void testSuccess();
  void testFailure();
  void testCancel();

  std::shared_ptr<rclcpp::Node> dummy_node_;
  std::unique_ptr<DummyPrimitive> primitive_;
}

// Finally, define the tests

void MotionPrimitivesTest::testSuccess()
{
}

void MotionPrimitivesTest::testFailure()
{
}

void MotionPrimitivesTest::testCancel()
{
}

TEST_F(MotionPrimitivesTest, allTests)
{
  testSuccess();
  testFailure();
  testCancel();
}
