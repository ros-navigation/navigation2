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

class LifecycleTransitionTestNode : public nav2_util::LifecycleNode
{
public:
  explicit LifecycleTransitionTestNode(rclcpp::NodeOptions options)
  : nav2_util::LifecycleNode("test_node", "", options) {}

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    configured = true;
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    activated = true;
    return nav2_util::CallbackReturn::SUCCESS;
  }

  bool configured{false};
  bool activated{false};
};

// For the following two tests, if the LifecycleNode doesn't shut down properly,
// the overall test will hang since the rclcpp thread will still be running,
// preventing the executable from exiting (the test will hang)

TEST(LifecycleNode, RclcppNodeExitsCleanly)
{
  // Make sure the node exits cleanly when using an rclcpp_node and associated thread
  auto node1 = std::make_shared<nav2_util::LifecycleNode>("test_node", "");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  SUCCEED();
}

TEST(LifecycleNode, MultipleRclcppNodesExitCleanly)
{
  // Try a couple nodes w/ rclcpp_node and threads
  auto node1 = std::make_shared<nav2_util::LifecycleNode>("test_node1", "");
  auto node2 = std::make_shared<nav2_util::LifecycleNode>("test_node2", "");

  std::this_thread::sleep_for(std::chrono::seconds(1));
  SUCCEED();
}

TEST(LifecycleNode, AutostartTransitions)
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::NodeOptions options;
  auto node = std::make_shared<LifecycleTransitionTestNode>(options);
  executor->add_node(node->get_node_base_interface());
  executor->spin_some();
  EXPECT_FALSE(node->configured);
  EXPECT_FALSE(node->activated);
  executor.reset();
  node.reset();


  executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  options.parameter_overrides({{"autostart_node", true}});
  node = std::make_shared<LifecycleTransitionTestNode>(options);
  executor->add_node(node->get_node_base_interface());
  executor->spin_some();
  EXPECT_TRUE(node->configured);
  EXPECT_TRUE(node->activated);
  executor.reset();
  node.reset();
}

TEST(LifecycleNode, OnPreshutdownCbFires)
{
  // Ensure the on_rcl_preshutdown_cb fires

  class MyNodeType : public nav2_util::LifecycleNode
  {
public:
    MyNodeType(
      const std::string & node_name)
    : nav2_util::LifecycleNode(node_name) {}

    bool fired = false;

protected:
    void on_rcl_preshutdown() override
    {
      fired = true;

      nav2_util::LifecycleNode::on_rcl_preshutdown();
    }
  };

  auto node = std::make_shared<MyNodeType>("test_node");

  ASSERT_EQ(node->fired, false);

  rclcpp::shutdown();

  ASSERT_EQ(node->fired, true);

  // Fire dtor to ensure nothing insane happens, e.g. exceptions.
  node.reset();

  SUCCEED();
}
