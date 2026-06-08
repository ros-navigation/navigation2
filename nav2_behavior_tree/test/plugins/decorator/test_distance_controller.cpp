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
#include <cmath>
#include <memory>
#include <set>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"

#include "utils/test_behavior_tree_fixture.hpp"
#include "utils/test_dummy_tree_node.hpp"
#include "nav2_behavior_tree/plugins/decorator/distance_controller.hpp"

class DistanceControllerTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue{0.1});
    config_->input_ports["distance"] = 1.0;
    config_->input_ports["global_frame"] = "map";
    config_->input_ports["robot_base_frame"] = "base_link";
    bt_node_ = std::make_shared<nav2_behavior_tree::DistanceController>(
      "distance_controller", *config_);
    dummy_node_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    bt_node_->setChild(dummy_node_.get());
  }

  void TearDown()
  {
    dummy_node_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::DistanceController> bt_node_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> dummy_node_;
};

std::shared_ptr<nav2_behavior_tree::DistanceController>
DistanceControllerTestFixture::bt_node_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
DistanceControllerTestFixture::dummy_node_ = nullptr;

class DistanceControllerGlobalTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue{0.1});
    config_->blackboard->set<std::string>("run_id", "run_1");
    config_->input_ports["distance"] = 1.0;
    config_->input_ports["global_frame"] = "map";
    config_->input_ports["robot_base_frame"] = "base_link";
    config_->input_ports["is_global"] = "true";
    bt_node_ = std::make_shared<nav2_behavior_tree::DistanceController>(
      "distance_controller", *config_);
    dummy_node_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    bt_node_->setChild(dummy_node_.get());
  }

  void TearDown()
  {
    dummy_node_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::DistanceController> bt_node_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> dummy_node_;
};

std::shared_ptr<nav2_behavior_tree::DistanceController>
DistanceControllerGlobalTestFixture::bt_node_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
DistanceControllerGlobalTestFixture::dummy_node_ = nullptr;

TEST_F(DistanceControllerTestFixture, test_behavior)
{
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);

  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.orientation.w = 1;

  double traveled = 0;
  for (int i = 1; i <= 20; i++) {
    pose.pose.position.x = i * 0.51;
    transform_handler_->updateRobotPose(pose.pose);

    // Wait for transforms to actually update
    // updated pose is i * 0.55
    // we wait for the traveled distance to reach a value > i * 0.5
    // we can assume the current transform has been updated at this point
    while (traveled < i * 0.5) {
      if (nav2_util::getCurrentPose(pose, *transform_handler_->getBuffer())) {
        traveled = std::sqrt(
          pose.pose.position.x * pose.pose.position.x +
          pose.pose.position.y * pose.pose.position.y);
      }
    }

    dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);

    if (i % 2) {
      EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
    } else {
      EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
      EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);
    }
  }
}

TEST_F(DistanceControllerGlobalTestFixture, test_global_runid_mode)
{
  // Move robot to origin
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 1.0;
  transform_handler_->updateRobotPose(pose.pose);

  // First tick: child gets ticked immediately (first_time_)
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // Distance not yet crossed: child should not tick
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);

  // Simulate halt by reactive parent — same run_id, start_pose_ must be preserved
  bt_node_->halt();
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);  // NOT ticked immediately

  // New RunID: start_pose_ resets to current position, move robot 1.1m to cross threshold
  config_->blackboard->set<std::string>("run_id", "run_2");
  pose.pose.position.x = 1.1;
  transform_handler_->updateRobotPose(pose.pose);
  double traveled = 0.0;
  while (traveled < 1.0) {
    geometry_msgs::msg::PoseStamped p;
    if (nav2_util::getCurrentPose(p, *transform_handler_->getBuffer())) {
      traveled = std::abs(p.pose.position.x);
    }
  }
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
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
