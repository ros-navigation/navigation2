// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2021 Samsung Research America
// Copyright (c) 2024 Marc Morcos
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

#include "behaviortree_cpp/bt_factory.h"

#include "utils/test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/get_pose_from_path_action.hpp"
#include "utils/test_behavior_tree_fixture.hpp"

class GetPoseFromPathTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("get_pose_from_path_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::GetPoseFromPath>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::GetPoseFromPath>(
      "GetPoseFromPath", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr GetPoseFromPathTestFixture::node_ = nullptr;
BT::NodeConfiguration * GetPoseFromPathTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> GetPoseFromPathTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> GetPoseFromPathTestFixture::tree_ = nullptr;

TEST_F(GetPoseFromPathTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <GetPoseFromPath path="{path}" pose="{pose}" index="{index}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new path and set it on blackboard
  nav_msgs::msg::Path path;
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  goals.resize(2);
  goals[0].pose.position.x = 1.0;
  goals[1].pose.position.x = 2.0;
  path.poses = goals;
  path.header.frame_id = "test_frame_1";
  config_->blackboard->set("path", path);

  config_->blackboard->set("index", 0);

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // the goal should have reached our server
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  // check if returned pose is correct
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_TRUE(config_->blackboard->get<geometry_msgs::msg::PoseStamped>("pose", pose));
  EXPECT_EQ(pose.header.frame_id, "test_frame_1");
  EXPECT_EQ(pose.pose.position.x, 1.0);

  // halt node so another goal can be sent
  tree_->haltTree();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);

  // try last element
  config_->blackboard->set("index", -1);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  // check if returned pose is correct
  EXPECT_TRUE(config_->blackboard->get<geometry_msgs::msg::PoseStamped>("pose", pose));
  EXPECT_EQ(pose.header.frame_id, "test_frame_1");
  EXPECT_EQ(pose.pose.position.x, 2.0);
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
