// Copyright (c) 2020 Samsung Research America
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
#include <memory>
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// minimal lifecycle node implementing bond as in rest of navigation servers
class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
  TestLifecycleNode(bool bond, std::string name)
  : nav2_util::LifecycleNode(name)
  {
    state = "";
    enable_bond = bond;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Configured!");
    state = "configured";
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Activated!");
    state = "activated";
    if (enable_bond) {
      createBond();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Deactivated!");
    state = "deactivated";
    if (enable_bond) {
      destroyBond();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Cleanup!");
    state = "cleaned up";
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Shutdown!");
    state = "shut down";
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is encountered an error!");
    state = "errored";
    return CallbackReturn::SUCCESS;
  }

  bool bondAllocated()
  {
    return bond_ ? true : false;
  }

  void breakBond()
  {
    bond_->breakBond();
  }

  std::string getState()
  {
    return state;
  }

  bool isBondEnabled()
  {
    return enable_bond;
  }

  bool isBondConnected()
  {
    return bondAllocated() ? !bond_->isBroken() : false;
  }

  std::string state;
  bool enable_bond;
};

class TestFixture
{
public:
  TestFixture(bool bond, std::string node_name)
  {
    lf_node_ = std::make_shared<TestLifecycleNode>(bond, node_name);
    lf_thread_ = std::make_unique<nav2_util::NodeThread>(lf_node_->get_node_base_interface());
  }

  std::shared_ptr<TestLifecycleNode> lf_node_;
  std::unique_ptr<nav2_util::NodeThread> lf_thread_;
};

TEST(LifecycleBondTest, POSITIVE)
{
  // let the lifecycle server come up
  rclcpp::Rate(1).sleep();

  nav2_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_test");

  // create node, should be up now
  auto fixture = TestFixture(true, "bond_tester");
  auto bond_tester = fixture.lf_node_;

  EXPECT_TRUE(client.startup());

  // check if bond is connected after being activated
  rclcpp::Rate(5).sleep();
  EXPECT_TRUE(bond_tester->isBondConnected());
  EXPECT_EQ(bond_tester->getState(), "activated");

  bond_tester->breakBond();

  // bond should be disconnected now and lifecycle manager should know and react to reset
  rclcpp::Rate(5).sleep();
  EXPECT_EQ(
    nav2_lifecycle_manager::SystemStatus::INACTIVE,
    client.is_active(std::chrono::nanoseconds(1000000000)));
  EXPECT_FALSE(bond_tester->isBondConnected());
  EXPECT_EQ(bond_tester->getState(), "cleaned up");

  // check that bringing up again is OK
  EXPECT_TRUE(client.startup());
  EXPECT_EQ(bond_tester->getState(), "activated");
  EXPECT_TRUE(bond_tester->isBondConnected());
  EXPECT_EQ(
    nav2_lifecycle_manager::SystemStatus::ACTIVE,
    client.is_active(std::chrono::nanoseconds(1000000000)));

  // clean state for next test.
  EXPECT_TRUE(client.reset());
  EXPECT_FALSE(bond_tester->isBondConnected());
  EXPECT_EQ(bond_tester->getState(), "cleaned up");
}

TEST(LifecycleBondTest, NEGATIVE)
{
  nav2_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_test");

  // create node, now without bond setup to connect to. Should fail because no bond
  auto fixture = TestFixture(false, "bond_tester");
  auto bond_tester = fixture.lf_node_;
  EXPECT_FALSE(client.startup());
  EXPECT_FALSE(bond_tester->isBondEnabled());
  EXPECT_EQ(
    nav2_lifecycle_manager::SystemStatus::INACTIVE,
    client.is_active(std::chrono::nanoseconds(1000000000)));
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
