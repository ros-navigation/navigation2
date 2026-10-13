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
#include <memory>

#include "utils/test_behavior_tree_fixture.hpp"
#include "utils/test_dummy_tree_node.hpp"
#include "nav2_behavior_tree/plugins/control/recovery_node.hpp"

// Changes status to SUCCESS after a specified number of failures
class RecoveryDummy : public nav2_behavior_tree::DummyNode
{
public:
  BT::NodeStatus tick() override
  {
    if (ticks_ == num_success_) {
      setStatus(BT::NodeStatus::SUCCESS);
    } else if (ticks_ == num_failure_) {
      setStatus(BT::NodeStatus::FAILURE);
    }
    if (ticks_ == num_running_) {
      setStatus(BT::NodeStatus::RUNNING);
    }
    ticks_++;
    return status();
  }

  void returnSuccessOn(int tick)
  {
    num_success_ = tick;
    ticks_ = 0;
  }

  void returnFailureOn(int tick)
  {
    num_failure_ = tick;
    ticks_ = 0;
  }

  void returnRunningOn(int tick)
  {
    num_running_ = tick;
  }

private:
  int num_running_{-1};
  int ticks_{0};
  int num_success_{-1};
  int num_failure_{-1};
};

class RecoveryNodeTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp() override
  {
    config_->input_ports.clear();
    config_->input_ports["number_of_retries"] = "1";
    bt_node_ = std::make_shared<nav2_behavior_tree::RecoveryNode>(
      "recovery_node", *config_);
    first_child_ = std::make_shared<RecoveryDummy>();
    second_child_ = std::make_shared<RecoveryDummy>();
    bt_node_->addChild(first_child_.get());
    bt_node_->addChild(second_child_.get());
  }

  void TearDown() override
  {
    first_child_.reset();
    second_child_.reset();
    bt_node_.reset();
  }

protected:
  void enableDistanceReset(double distance = 1.0)
  {
    // Use a private buffer and direct transforms to keep these tests deterministic.
    distance_tf_ = nav2::create_transform_buffer(node_);
    auto config = *config_;
    config.blackboard = BT::Blackboard::create();
    config.blackboard->set("node", node_);
    config.blackboard->set("tf_buffer", distance_tf_);
    config.input_ports["reset_distance"] = std::to_string(distance);
    config.input_ports["robot_frame"] = "test_robot";
    config.input_ports["global_frame"] = "test_world";
    bt_node_ = std::make_shared<nav2_behavior_tree::RecoveryNode>("recovery_node", config);
    bt_node_->addChild(first_child_.get());
    bt_node_->addChild(second_child_.get());
  }

  void moveRobot(double x, double y = 0.0)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "test_world";
    transform.child_frame_id = "test_robot";
    transform.header.stamp = node_->now();
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(distance_tf_->setTransform(transform, "test", true));
  }

  BT::NodeStatus retryOnce()
  {
    first_child_->returnFailureOn(0);
    first_child_->returnRunningOn(1);
    first_child_->changeStatus(BT::NodeStatus::FAILURE);
    second_child_->changeStatus(BT::NodeStatus::SUCCESS);
    return bt_node_->executeTick();
  }

  nav2::TransformBuffer::SharedPtr distance_tf_;
  static std::shared_ptr<nav2_behavior_tree::RecoveryNode> bt_node_;
  static std::shared_ptr<RecoveryDummy> first_child_;
  static std::shared_ptr<RecoveryDummy> second_child_;
};

std::shared_ptr<nav2_behavior_tree::RecoveryNode> RecoveryNodeTestFixture::bt_node_ = nullptr;
std::shared_ptr<RecoveryDummy> RecoveryNodeTestFixture::first_child_ = nullptr;
std::shared_ptr<RecoveryDummy> RecoveryNodeTestFixture::second_child_ = nullptr;

TEST_F(RecoveryNodeTestFixture, test_only_two_children)
{
  nav2_behavior_tree::DummyNode dummy;
  bt_node_->addChild(&dummy);
  EXPECT_THROW(bt_node_->executeTick(), BT::BehaviorTreeException);
}

TEST_F(RecoveryNodeTestFixture, test_running)
{
  first_child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
}

TEST_F(RecoveryNodeTestFixture, test_failure_on_idle_child)
{
  first_child_->changeStatus(BT::NodeStatus::IDLE);
  EXPECT_THROW(bt_node_->executeTick(), BT::NodeExecutionError);
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::IDLE);
  EXPECT_THROW(bt_node_->executeTick(), BT::NodeExecutionError);
}

TEST_F(RecoveryNodeTestFixture, test_success_one_retry)
{
  // first child returns success right away
  first_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // first child fails, second child succeeds, then first child succeeds (one retry)
  first_child_->returnSuccessOn(1);
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
}

TEST_F(RecoveryNodeTestFixture, test_failure_one_retry)
{
  // first child fails, second child fails
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);

  // first child fails, second child succeeds, then first child fails (one retry)
  first_child_->returnFailureOn(1);
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
}

TEST_F(RecoveryNodeTestFixture, test_skipping)
{
  // first child skipped
  first_child_->changeStatus(BT::NodeStatus::SKIPPED);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SKIPPED);
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);

  // first child fails, second child skipped
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::SKIPPED);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
}


TEST_F(RecoveryNodeTestFixture, test_distance_below_threshold)
{
  enableDistanceReset();
  moveRobot(0.0);
  EXPECT_EQ(retryOnce(), BT::NodeStatus::RUNNING);
  moveRobot(0.6);
  EXPECT_EQ(retryOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(RecoveryNodeTestFixture, test_distance_threshold_and_repeated_resets)
{
  enableDistanceReset();
  moveRobot(0.0);
  EXPECT_EQ(retryOnce(), BT::NodeStatus::RUNNING);
  // Both axes contribute, and equality with the threshold resets the count.
  moveRobot(0.6, 0.8);
  EXPECT_EQ(retryOnce(), BT::NodeStatus::RUNNING);
  moveRobot(0.6, 2.0);
  EXPECT_EQ(retryOnce(), BT::NodeStatus::RUNNING);
  moveRobot(0.6, 2.5);
  EXPECT_EQ(retryOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(RecoveryNodeTestFixture, test_halt_clears_distance_reference)
{
  enableDistanceReset();
  moveRobot(0.0);
  EXPECT_EQ(retryOnce(), BT::NodeStatus::RUNNING);
  bt_node_->haltNode();
  moveRobot(10.0);
  EXPECT_EQ(retryOnce(), BT::NodeStatus::RUNNING);
  moveRobot(10.5);
  EXPECT_EQ(retryOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(RecoveryNodeTestFixture, test_missing_transform_preserves_retry_limit)
{
  enableDistanceReset();
  moveRobot(0.0);
  EXPECT_EQ(retryOnce(), BT::NodeStatus::RUNNING);
  distance_tf_->clear();
  EXPECT_EQ(retryOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(RecoveryNodeTestFixture, test_initially_missing_transform)
{
  enableDistanceReset();
  EXPECT_EQ(retryOnce(), BT::NodeStatus::RUNNING);
  moveRobot(10.0);
  // Acquiring a reference without measured progress does not restore retries.
  EXPECT_EQ(retryOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(RecoveryNodeTestFixture, test_nonpositive_distance_needs_no_blackboard_dependencies)
{
  auto config = *config_;
  config.blackboard = BT::Blackboard::create();
  for (const auto & distance : {"0.0", "-1.0"}) {
    config.input_ports["reset_distance"] = distance;
    bt_node_ = std::make_shared<nav2_behavior_tree::RecoveryNode>("recovery_node", config);
    bt_node_->addChild(first_child_.get());
    bt_node_->addChild(second_child_.get());
    EXPECT_EQ(retryOnce(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(retryOnce(), BT::NodeStatus::FAILURE);
  }
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
