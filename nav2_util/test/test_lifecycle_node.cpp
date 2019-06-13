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
// limitations under the License.

#include <memory>

#include "gtest/gtest.h"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

// For the following two tests, if the LifecycleNode doesn't shut down properly,
// the overall test will hang since the rclcpp thread will still be running,
// preventing the executable from exiting (the test will hang)

TEST(LifecycleNode, RclcppNodeExitsCleanly)
{
  // Make sure the node exits cleanly when using an rclcpp_node and associated thread
  auto node1 = std::make_shared<nav2_util::LifecycleNode>("test_node", "", true);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  SUCCEED();
}

TEST(LifecycleNode, MultipleRclcppNodesExitCleanly)
{
  // Try a couple nodes w/ rclcpp_node and threads
  auto node1 = std::make_shared<nav2_util::LifecycleNode>("test_node1", "", true);
  auto node2 = std::make_shared<nav2_util::LifecycleNode>("test_node2", "", true);

  // Another one without rclcpp_node and on the stack
  nav2_util::LifecycleNode node3("test_node3", "", false);

  std::this_thread::sleep_for(std::chrono::seconds(1));
  SUCCEED();
}
