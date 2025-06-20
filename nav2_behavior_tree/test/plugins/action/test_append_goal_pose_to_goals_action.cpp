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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/append_goal_pose_to_goals_action.hpp"
#include "utils/test_behavior_tree_fixture.hpp"

class AppendGoalPoseToGoalsTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();
    transform_handler_ = std::make_shared<nav2_behavior_tree::TransformHandler>(node_);
    transform_handler_->activate();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set(
      "node",
      node_);
    config_->blackboard->set(
      "tf_buffer",
      transform_handler_->getBuffer());

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::AppendGoalPoseToGoals>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::AppendGoalPoseToGoals>(
      "AppendGoalPoseToGoals", builder);
  }

  static void TearDownTestCase()
  {
    transform_handler_->deactivate();
    delete config_;
    config_ = nullptr;
    transform_handler_.reset();
    node_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
  static std::shared_ptr<nav2_behavior_tree::TransformHandler> transform_handler_;
};

nav2::LifecycleNode::SharedPtr AppendGoalPoseToGoalsTestFixture::node_ = nullptr;

BT::NodeConfiguration * AppendGoalPoseToGoalsTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> AppendGoalPoseToGoalsTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> AppendGoalPoseToGoalsTestFixture::tree_ = nullptr;
std::shared_ptr<nav2_behavior_tree::TransformHandler>
AppendGoalPoseToGoalsTestFixture::transform_handler_ = nullptr;

TEST_F(AppendGoalPoseToGoalsTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <AppendGoalPoseToGoals input_goals="{goals}" output_goals="{goals}" goal_pose="{pose}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new goal and set it on blackboard
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node_->now();
  pose.pose.position.x = 10;
  config_->blackboard->set("pose", pose);

  nav_msgs::msg::Goals input_goals;
  config_->blackboard->set("goals", input_goals);

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // check that it removed the point in range
  nav_msgs::msg::Goals output_goals;
  EXPECT_TRUE(config_->blackboard->get("goals", output_goals));

  EXPECT_EQ(output_goals.goals.size(), 1u);
  EXPECT_EQ(output_goals.goals[0].header.frame_id, "map");
  EXPECT_EQ(output_goals.goals[0].pose.position.x, 10.0);
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
