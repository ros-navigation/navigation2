// Copyright (c) 2025 Open Navigation LLC
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
#include "nav2_behavior_tree/plugins/condition/are_poses_near_condition.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class ArePosesNearConditionTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
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
    config_->blackboard->set("p1", goal);

    std::string xml_txt =
      R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <ArePosesNear ref_pose="{p1}" target_pose="{p2}" tolerance="0.5"/>
        </BehaviorTree>
      </root>)";

    factory_->registerNodeType<nav2_behavior_tree::ArePosesNearCondition>("ArePosesNear");
    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  }

  void TearDown()
  {
    tree_.reset();
  }

protected:
  static std::shared_ptr<BT::Tree> tree_;
};

std::shared_ptr<BT::Tree> ArePosesNearConditionTestFixture::tree_ = nullptr;

TEST_F(ArePosesNearConditionTestFixture, test_behavior)
{
  geometry_msgs::msg::PoseStamped p2;
  p2.header.stamp = node_->now();
  p2.header.frame_id = "map";
  config_->blackboard->set("p2", p2);
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::FAILURE);

  p2.pose.position.x = 1.0;
  p2.pose.position.y = 1.0;
  config_->blackboard->set("p2", p2);
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::SUCCESS);

  p2.pose.position.x = 1.1;
  p2.pose.position.y = 1.1;
  config_->blackboard->set("p2", p2);
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
