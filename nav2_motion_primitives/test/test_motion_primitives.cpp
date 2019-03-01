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
#include <atomic>
#include <iostream>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_tasks/task_client.hpp"
#include "nav2_tasks/task_status.hpp"
#include "nav2_motion_primitives/motion_primitive.hpp"

using nav2_tasks::TaskStatus;
using nav2_motion_primitives::MotionPrimitive;
using namespace std::chrono_literals;

// Let's create a motion primitive for testing the base class

using DummyPrimitiveCommand = std_msgs::msg::String;
using DummyPrimitiveResult = std_msgs::msg::String;

using DummyPrimitiveClient = nav2_tasks::TaskClient<DummyPrimitiveCommand, DummyPrimitiveResult>;

class DummyPrimitive : public MotionPrimitive<DummyPrimitiveCommand, DummyPrimitiveResult>
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

    // onRun method can have various possible outcomes (success, failure, cancelled)
    // The output is defined by the tester class on the command string.

    if (command_->data == "Testing failure on init") {
      // A real primitive would return failed if initialization failed or command is invalid
      return TaskStatus::FAILED;
      // return TaskStatus::SUCCEEDED;
    } else if (command_->data == "Testing success" || command->data == "Testing failure on run") {
      initialized_ = true;
      return TaskStatus::SUCCEEDED;
    } else {
      return TaskStatus::FAILED;
    }
  }

  TaskStatus onCycleUpdate(DummyPrimitiveResult & /*result*/) override
  {
    // A normal primitive would set the robot in motion in the first call
    // and check for robot states on subsequent calls to check if the movement
    // was completed.

    if (command_->data != "Testing success" || !initialized_) {
      return TaskStatus::FAILED;
    }

    // Fake getting the robot state, calculate and send control output
    std::this_thread::sleep_for(2ms);

    // For testing, pretend the robot takes some fixed
    // amount of time to complete the motion.
    auto current_time = std::chrono::system_clock::now();
    auto motion_duration = 5s;

    if (current_time - start_time_ >= motion_duration) {
      // Movement was completed
      return TaskStatus::SUCCEEDED;
    }

    return TaskStatus::RUNNING;
  }

private:
  bool initialized_;
  std::shared_ptr<DummyPrimitiveCommand> command_;
  std::chrono::system_clock::time_point start_time_;
};

// The following getTaskName function is required and currently has to be
// in the nav2_tasks namespace

namespace nav2_tasks
{

template<>
inline const char * getTaskName<DummyPrimitiveCommand, DummyPrimitiveResult>()
{
  return "TestMotionPrimitivesTask";
}

}

// Define a test class to hold the context for the tests

class MotionPrimitivesTest : public ::testing::Test
{
protected:
  MotionPrimitivesTest()
  : spinning_ok_(false) {}

  ~MotionPrimitivesTest() {}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("MotionPrimitivesTestNode");

    primitive_ = std::make_unique<DummyPrimitive>(node_);

    client_ = std::make_unique<DummyPrimitiveClient>(node_);

    // Launch a thread to spin the node
    spinning_ok_ = true;
    spin_thread_ = new std::thread(&MotionPrimitivesTest::spin, this);
  }

  void TearDown() override
  {
    spinning_ok_ = false;
    spin_thread_->join();
    delete spin_thread_;
  }

  void spin()
  {
    while (spinning_ok_) {
      // Spin the node to get messages from the subscriptions
      rclcpp::spin_some(node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Exiting node spinning function" << std::endl;
  }

  void sendStringCommand(const std::string & string_command)
  {
    auto command = std::make_shared<DummyPrimitiveCommand>();
    command->data = string_command;
    client_->sendCommand(command);
  }

  TaskStatus waitForPrimitive()
  {
    auto result = std::make_shared<DummyPrimitiveResult>();

    // Loop until primitive is completed
    while (true) {
      TaskStatus status = client_->waitForResult(result, 100ms);

      if (status != TaskStatus::RUNNING) {
        return status;
      }
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<DummyPrimitive> primitive_;
  std::unique_ptr<DummyPrimitiveClient> client_;
  std::thread * spin_thread_;
  std::atomic<bool> spinning_ok_;
};

// Define the tests

TEST_F(MotionPrimitivesTest, testingSuccess)
{
  sendStringCommand("Testing success");
  EXPECT_EQ(waitForPrimitive(), TaskStatus::SUCCEEDED);
}

TEST_F(MotionPrimitivesTest, testingFailureOnRun)
{
  sendStringCommand("Testing failure on run");
  EXPECT_EQ(waitForPrimitive(), TaskStatus::FAILED);
}

TEST_F(MotionPrimitivesTest, testingFailureOnInit)
{
  sendStringCommand("Testing failure on init");
  EXPECT_EQ(waitForPrimitive(), TaskStatus::FAILED);
}

TEST_F(MotionPrimitivesTest, testingSequentialFailures)
{
  sendStringCommand("Testing failure on init");
  EXPECT_EQ(waitForPrimitive(), TaskStatus::FAILED);

  sendStringCommand("Testing failure on run");
  EXPECT_EQ(waitForPrimitive(), TaskStatus::FAILED);
}

TEST_F(MotionPrimitivesTest, testCancel)
{
  sendStringCommand("Testing success");
  client_->cancel();
  EXPECT_EQ(waitForPrimitive(), TaskStatus::CANCELED);
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
