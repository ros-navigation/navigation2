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
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/goal_reached_condition.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class GoalReachedConditionTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue{0.1});

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = node_->now();
    goal.header.frame_id = "map";
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 1.0;
    config_->blackboard->set("goal", goal);

    std::string xml_txt =
      R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <GoalReached goal="{goal}" robot_base_frame="base_link"/>
        </BehaviorTree>
      </root>)";

    factory_->registerNodeType<nav2_behavior_tree::GoalReachedCondition>("GoalReached");
    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  }

  void TearDown()
  {
    tree_.reset();
  }

protected:
  static std::shared_ptr<BT::Tree> tree_;
};

std::shared_ptr<BT::Tree> GoalReachedConditionTestFixture::tree_ = nullptr;

TEST_F(GoalReachedConditionTestFixture, test_behavior)
{
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::FAILURE);

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  transform_handler_->updateRobotPose(pose);
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::FAILURE);

  pose.position.x = 0.5;
  pose.position.y = 0.5;
  transform_handler_->updateRobotPose(pose);
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::FAILURE);

  pose.position.x = 0.9;
  pose.position.y = 0.9;
  transform_handler_->updateRobotPose(pose);
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::SUCCESS);

  pose.position.x = 1.0;
  pose.position.y = 1.0;
  transform_handler_->updateRobotPose(pose);
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::SUCCESS);
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
