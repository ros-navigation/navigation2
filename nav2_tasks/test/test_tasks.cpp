// Copyright (c) 2018 Intel Corporation
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

#include <string>
#include <memory>
#include <algorithm>
#include "gtest/gtest.h"
#include "std_msgs/msg/string.hpp"
#include "nav2_tasks/task_client.hpp"
#include "nav2_tasks/task_server.hpp"

using namespace std::chrono_literals;
using nav2_tasks::TaskStatus;

// A global object to initialize ROS

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
};

RclCppFixture g_rclcppfixture;

// Define a task client/server pair for this test

using TestCommand = std_msgs::msg::String;
using TestResult = std_msgs::msg::String;

using TestTaskClient = nav2_tasks::TaskClient<TestCommand, TestResult>;
using TestTaskServer = nav2_tasks::TaskServer<TestCommand, TestResult>;

// Implement the task server by defining a class and overriding the
// virtual execute method

class MyTestTaskServer : public TestTaskServer
{
public:
  explicit MyTestTaskServer(rclcpp::Node::SharedPtr & node)
  : TestTaskServer(node)
  {
  }

  MyTestTaskServer() = delete;

  TaskStatus execute(const TestCommand::SharedPtr command)
  {
    // A normal task would have various possible outcomes (success,
    // failure, canceled). Since we're trying to cause the task server
    // to generate the various kinds of returns, we'll use the command
    // string to communicate so that the server can do the right thing
    if (command->data == "Testing failure") {
      // Test if the failure code is propagated back to the client
      return TaskStatus::FAILED;
    } else if (command->data == "Testing cancel") {
      // We'll enter a loop so that the client can cancel the task
      for (;; ) {
        if (cancelRequested()) {
          setCanceled();
          return TaskStatus::CANCELED;
        }
        std::this_thread::sleep_for(10ms);
      }
    } else {
      // Otherwise, let's perform some service, such as reversing
      // the input string
      TestResult result;
      result.data = command->data;
      std::reverse(result.data.begin(), result.data.end());

      setResult(result);
      return TaskStatus::SUCCEEDED;
    }
  }
};

// The following getTaskName function is required and currently has to be
// in the nav2_tasks namespace

namespace nav2_tasks
{

template<>
inline const char * getTaskName<TestCommand, TestResult>()
{
  return "TestTask";
}

}

// Define a test class to hold the context for the tests

class TaskClientServerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("TestNode");

    // The task client is not itself a node, so needs a node to use
    client_ = std::make_shared<TestTaskClient>(node_);

    // Same for the task server
    server_ = std::make_shared<MyTestTaskServer>(node_);

    // A task server must have its execute callback set
    server_->setExecuteCallback(
      std::bind(&MyTestTaskServer::execute, server_, std::placeholders::_1));

    // Launch a thread to spin the node
    spin_thread_ = new std::thread(&TaskClientServerTest::spin, this);

    // After creating the nodes, there is a lot of multicast traffic that can
    // cause nodes to miss messages. Let's sleep for a bit to let the nodes settle
    // out and process their incoming message queues. We might need to use managed
    // nodes to avoid this kind of thing
    std::this_thread::sleep_for(2s);
  }

  void TearDown() override
  {
    spin_thread_->join();
    delete spin_thread_;
  }

  void spin()
  {
    rclcpp::spin(node_);
  }

  void testSuccess();
  void testFailure();
  void testCancel();

  std::thread * spin_thread_;
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<TestTaskClient> client_;
  std::shared_ptr<MyTestTaskServer> server_;
};

// Finally, we can get to some tests

void TaskClientServerTest::testSuccess()
{
  auto command = std::make_shared<TestCommand>();
  command->data = "Hello, World!";

  auto result = std::make_shared<TestResult>();

  client_->sendCommand(command);

  for (;; ) {
    TaskStatus status = client_->waitForResult(result, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        {
          std::string expected_result("!dlroW ,olleH");
          EXPECT_EQ(result->data, expected_result);
          return;
        }

      case TaskStatus::RUNNING:
        break;

      case TaskStatus::FAILED:
        ADD_FAILURE() << "Task failed, but expected to succeed";
        return;

      case TaskStatus::CANCELED:
        ADD_FAILURE() << "Task canceled, but expected to succeed";
        return;

      default:
        ADD_FAILURE() << "Invalid task return value";
        return;
    }
  }
}

void TaskClientServerTest::testFailure()
{
  auto command = std::make_shared<TestCommand>();
  command->data = "Testing failure";

  auto result = std::make_shared<TestResult>();

  client_->sendCommand(command);

  for (;; ) {
    TaskStatus status = client_->waitForResult(result, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        ADD_FAILURE() << "Task succeeded, but expected to fail";
        return;

      case TaskStatus::RUNNING:
        break;

      case TaskStatus::FAILED:
        // We've received the correct return code. Let's add a check so
        // there is some output for this test
        EXPECT_EQ(status, TaskStatus::FAILED);
        return;

      case TaskStatus::CANCELED:
        ADD_FAILURE() << "Task canceled, but expected to fail";
        return;

      default:
        ADD_FAILURE() << "Invalid task return value";
        return;
    }
  }
}

void TaskClientServerTest::testCancel()
{
  auto command = std::make_shared<TestCommand>();
  command->data = "Testing cancel";

  auto result = std::make_shared<TestResult>();

  client_->sendCommand(command);

  // Let's assume the client cancels the command after a bit
  std::this_thread::sleep_for(100ms);
  client_->cancel();

  for (;; ) {
    TaskStatus status = client_->waitForResult(result, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        ADD_FAILURE() << "Task succeeded, but expected canceled";
        return;

      case TaskStatus::RUNNING:
        break;

      case TaskStatus::FAILED:
        ADD_FAILURE() << "Task failed, but expected canceled";
        return;

      case TaskStatus::CANCELED:
        // We've received the correct return code. Let's add a check so
        // there is some output for this test
        EXPECT_EQ(status, TaskStatus::CANCELED);
        return;

      default:
        ADD_FAILURE() << "Invalid task return value";
        return;
    }
  }
}

TEST_F(TaskClientServerTest, allTests)
{
  testSuccess();
  testFailure();
  testCancel();

  // Invoke shutdown here so that the task server exits
  // and the test terminates without hanging
  rclcpp::shutdown();
}
