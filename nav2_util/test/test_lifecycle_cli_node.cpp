// Copyright (c) 2020 Samsung Research
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

#ifndef NAV2_UTIL__TEST__TEST_LIFECYCLE_CLI_NODE_HPP_
#define NAV2_UTIL__TEST__TEST_LIFECYCLE_CLI_NODE_HPP_

#include <cstdlib>
#include <memory>
#include "gtest/gtest.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/lifecycle_utils.hpp"
#include "nav2_util/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

class DummyNode : public nav2_util::LifecycleNode
{
public:
  DummyNode()
  : nav2_util::LifecycleNode("nav2_test_cli", "")
  {
    activated = false;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/)
  {
    activated = true;
    return nav2_util::CallbackReturn::SUCCESS;
  }

  bool activated;
};

class Handle
{
public:
  Handle()
  {
    node = std::make_shared<DummyNode>();
    thread = std::make_shared<nav2_util::NodeThread>(node->get_node_base_interface());
  }
  ~Handle()
  {
    thread.reset();
    node.reset();
  }

  std::shared_ptr<nav2_util::NodeThread> thread;
  std::shared_ptr<DummyNode> node;
};

class RclCppFixture
{
public:
  RclCppFixture()
  {
    rclcpp::init(0, nullptr);
  }

  ~RclCppFixture()
  {
    rclcpp::shutdown();
  }
};

RclCppFixture g_rclcppfixture;

TEST(LifeycleCLI, fails_no_node_name)
{
  Handle handle;
  auto rc = system("ros2 run nav2_util lifecycle_bringup");
  (void)rc;
#ifdef _WIN32
  Sleep(1000);
#else
  sleep(1);
#endif
  // check node didn't mode
  EXPECT_EQ(handle.node->activated, false);
  SUCCEED();
}

TEST(LifeycleCLI, succeeds_node_name)
{
  Handle handle;
  auto rc = system("ros2 run nav2_util lifecycle_bringup nav2_test_cli");
#ifdef _WIN32
  Sleep(3000);
#else
  sleep(3);
#endif
  // check node moved
  (void)rc;
  EXPECT_EQ(handle.node->activated, true);
  SUCCEED();
}

#endif  // NAV2_UTIL__TEST__TEST_LIFECYCLE_CLI_NODE_HPP_
