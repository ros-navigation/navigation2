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
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/goals.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/get_current_pose_action.hpp"
#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_ros_common/node_utils.hpp"

class GetCurrentPoseTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    config_->blackboard->set("robot_base_frame", "base_link");
    config_->blackboard->set("global_frame", "map");
    nav2::declare_parameter_if_not_declared(
      node_, "robot_base_frame", rclcpp::ParameterValue("base_link"));
    nav2::declare_parameter_if_not_declared(
      node_, "global_frame", rclcpp::ParameterValue("map"));

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::GetCurrentPoseAction>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::GetCurrentPoseAction>(
      "GetCurrentPose", builder);
  }

  void TearDown()
  {
    tree_.reset();
  }

protected:
  static std::shared_ptr<BT::Tree> tree_;
};

std::shared_ptr<BT::Tree> GetCurrentPoseTestFixture::tree_ = nullptr;

TEST_F(GetCurrentPoseTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <GetCurrentPose current_pose="{current_pose}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  transform_handler_->updateRobotPose(pose);
  std::this_thread::sleep_for(500ms);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // Check the output
  geometry_msgs::msg::PoseStamped current_pose;
  EXPECT_TRUE(config_->blackboard->get("current_pose", current_pose));
  EXPECT_EQ(current_pose.pose.position.x, pose.position.x);
  EXPECT_EQ(current_pose.pose.position.y, pose.position.y);
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
