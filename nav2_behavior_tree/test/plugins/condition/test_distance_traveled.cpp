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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/distance_traveled_condition.hpp"

class DistanceTraveledConditionTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    config_->input_ports["global_frame"] = "map";
    config_->input_ports["robot_base_frame"] = "base_link";
    config_->input_ports["distance"] = 1.0;
    bt_node_ = std::make_shared<nav2_behavior_tree::DistanceTraveledCondition>(
      "distance_traveled", *config_);
  }

  void TearDown()
  {
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::DistanceTraveledCondition> bt_node_;
};

std::shared_ptr<nav2_behavior_tree::DistanceTraveledCondition>
DistanceTraveledConditionTestFixture::bt_node_ = nullptr;

TEST_F(DistanceTraveledConditionTestFixture, test_behavior)
{
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.orientation.w = 1;

  double traveled = 0;
  for (int i = 1; i <= 20; i++) {
    pose.pose.position.x = i * 0.51;
    transform_handler_->updateRobotPose(pose.pose);

    // Wait for transforms to actually update
    // updated pose is i * 0.51
    // we wait for the traveled distance to reach a value > i * 0.5
    // we can assume the current transform has been updated at this point
    while (traveled < i * 0.5) {
      if (nav2_util::getCurrentPose(pose, *transform_handler_->getBuffer())) {
        traveled = pose.pose.position.x;
      }
    }

    if (i % 2) {
      EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
    } else {
      EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
    }
  }
}

TEST_F(DistanceTraveledConditionTestFixture, test_runid_global_mode)
{
  // Reset robot to origin so we don't inherit stale position from test_behavior
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 1.0;
  transform_handler_->updateRobotPose(pose.pose);
  geometry_msgs::msg::PoseStamped current_pose;
  double current_x = 1.0;
  while (current_x > 0.05) {
    if (nav2_util::getCurrentPose(current_pose, *transform_handler_->getBuffer())) {
      current_x = current_pose.pose.position.x;
    }
  }

  // Enable global mode
  node_->declare_or_get_parameter("is_global", false);
  node_->set_parameter(rclcpp::Parameter("is_global", true));
  config_->blackboard->set<std::string>("run_id", "runid_1");

  // First tick: initialize start_pose_ at position 0
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // Move robot to 0.6 — not enough distance yet
  pose.pose.position.x = 0.6;
  transform_handler_->updateRobotPose(pose.pose);
  current_x = 0;
  while (current_x < 0.5) {
    if (nav2_util::getCurrentPose(current_pose, *transform_handler_->getBuffer())) {
      current_x = current_pose.pose.position.x;
    }
  }
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // New RunID: start_pose_ must NOT reset — distance still accumulates from 0
  config_->blackboard->set<std::string>("run_id", "runid_2");
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // Move to 1.1 — now > 1.0 from start (position 0), should succeed
  pose.pose.position.x = 1.1;
  transform_handler_->updateRobotPose(pose.pose);
  current_x = 0;
  while (current_x < 1.0) {
    if (nav2_util::getCurrentPose(current_pose, *transform_handler_->getBuffer())) {
      current_x = current_pose.pose.position.x;
    }
  }
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
