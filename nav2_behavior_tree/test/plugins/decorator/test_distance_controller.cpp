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
#include <set>

#include "nav2_ros_common/tf_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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

TEST_F(DistanceControllerTestFixture, test_behavior)
{
  // Stop the periodic broadcaster so the directly seeded poses remain authoritative.
  transform_handler_->deactivate();
  const auto buffer = transform_handler_->getBuffer();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node_->now();
  pose.pose.orientation.w = 1.0;
  ASSERT_TRUE(buffer->setTransform(nav2::poseToTransformStamped(pose, "base_link"), "test", false));

  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);

  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);

  for (int i = 1; i <= 20; i++) {
    pose.pose.position.x = i * 0.51;
    pose.header.stamp = node_->now();
    ASSERT_TRUE(buffer->setTransform(nav2::poseToTransformStamped(pose, "base_link"), "test",
      false));
    const auto current_pose = nav2::getLatestPose(*buffer, "map", "base_link");
    ASSERT_TRUE(current_pose);
    EXPECT_DOUBLE_EQ(current_pose->pose.position.x, pose.pose.position.x);

    dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);

    if (i % 2) {
      EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
    } else {
      EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
      EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);
    }
  }
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
