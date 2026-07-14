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
#include <chrono>
#include <memory>
#include <set>

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/decorator/rate_controller.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class RateControllerTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    config_->input_ports["hz"] = 10.0;
    bt_node_ = std::make_shared<nav2_behavior_tree::RateController>(
      "rate_controller", *config_);
    dummy_node_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    bt_node_->setChild(dummy_node_.get());
  }

  void TearDown()
  {
    dummy_node_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::RateController> bt_node_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> dummy_node_;
};

std::shared_ptr<nav2_behavior_tree::RateController>
RateControllerTestFixture::bt_node_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
RateControllerTestFixture::dummy_node_ = nullptr;

// Shim to expose resetStatus for halt simulation
class RateControllerShim : public nav2_behavior_tree::RateController
{
public:
  RateControllerShim(const std::string & name, const BT::NodeConfiguration & conf)
  : RateController(name, conf) {}
  void changeStatus() {resetStatus();}
};

class RateControllerGlobalTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    config_->blackboard->set<std::string>("run_id", "run_1");
    config_->input_ports["hz"] = "1.0";
    config_->input_ports["is_global"] = "true";
    bt_node_ = std::make_shared<RateControllerShim>("rate_controller", *config_);
    dummy_node_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    bt_node_->setChild(dummy_node_.get());
  }

  void TearDown()
  {
    dummy_node_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<RateControllerShim> bt_node_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> dummy_node_;
};

std::shared_ptr<RateControllerShim>
RateControllerGlobalTestFixture::bt_node_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
RateControllerGlobalTestFixture::dummy_node_ = nullptr;

TEST_F(RateControllerTestFixture, test_behavior)
{
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);

  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);

  for (int i = 0; i < 10; ++i) {
    dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
    std::this_thread::sleep_for(500ms);
    if (i % 2) {
      EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
      EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);
    } else {
      EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
    }
  }
}

TEST_F(RateControllerGlobalTestFixture, test_global_runid_mode)
{
  // First tick: child gets ticked immediately (first_time_)
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // Period not elapsed: child should not tick
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);

  // Simulate halt by reactive parent — same run_id, timer must be preserved
  bt_node_->halt();
  bt_node_->changeStatus();
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);  // NOT ticked immediately

  // New RunID: timer resets, child should tick immediately
  config_->blackboard->set<std::string>("run_id", "run_2");
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // Period not elapsed again: child should not tick
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
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
