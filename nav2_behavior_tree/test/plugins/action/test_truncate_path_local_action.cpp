// Copyright (c) 2021 RoboTech Vision
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

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/action/truncate_path_local_action.hpp"


class TruncatePathLocalTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp() override
  {
    bt_node_ = std::make_shared<nav2_behavior_tree::TruncatePathLocal>(
      "truncate_path_local", *config_);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::TruncatePathLocal>(
          name, config);
      };
    try {
      factory_->registerBuilder<nav2_behavior_tree::TruncatePathLocal>(
        "TruncatePathLocal", builder);
    } catch (BT::BehaviorTreeException const &) {
      // ignoring multiple registrations of TruncatePathLocal
    }
  }

  void TearDown() override
  {
    bt_node_.reset();
    tree_.reset();
  }

  static geometry_msgs::msg::PoseStamped poseMsg(double x, double y, double orientation)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(orientation);
    return pose;
  }

  nav_msgs::msg::Path createLoopCrossingTestPath()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = "map";

    // this is a loop to make it harder for robot to find the proper closest pose
    path.poses.push_back(poseMsg(-0.3, -1.2, -M_PI * 3 / 2));
    // the position is closest to robot but orientation is different
    path.poses.push_back(poseMsg(-0.3, 0.0, -M_PI * 3 / 2));
    path.poses.push_back(poseMsg(-0.5, 1.0, -M_PI));
    path.poses.push_back(poseMsg(-1.5, 1.0, -M_PI / 2));
    path.poses.push_back(poseMsg(-1.5, 0.0, 0.0));

    // this is the correct path section for the first match
    path.poses.push_back(poseMsg(-0.5, 0.0, 0.0));
    path.poses.push_back(poseMsg(0.4, 0.0, 0.0));
    path.poses.push_back(poseMsg(1.5, 0.0, 0.0));

    // this is a loop to make it harder for robot to find the proper closest pose
    path.poses.push_back(poseMsg(1.5, 1.0, M_PI / 2));
    path.poses.push_back(poseMsg(0.5, 1.0, M_PI));
    // the position is closest to robot but orientation is different
    path.poses.push_back(poseMsg(0.3, 0.0, M_PI * 3 / 2));
    path.poses.push_back(poseMsg(0.3, -1.0, M_PI * 3 / 2));

    return path;
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::TruncatePathLocal> bt_node_;
  static std::shared_ptr<BT::Tree> tree_;
};

std::shared_ptr<nav2_behavior_tree::TruncatePathLocal> TruncatePathLocalTestFixture::bt_node_ =
  nullptr;
std::shared_ptr<BT::Tree> TruncatePathLocalTestFixture::tree_ = nullptr;

TEST_F(TruncatePathLocalTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <TruncatePathLocal
            distance_forward="2.0"
            distance_backward="1.0"
            robot_frame="base_link"
            transform_tolerance="0.2"
            angular_distance_weight="0.2"
            pose="{pose}"
            input_path="{path}"
            output_path="{truncated_path}"
          />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create path and set it on blackboard
  nav_msgs::msg::Path path = createLoopCrossingTestPath();
  EXPECT_EQ(path.poses.size(), 12u);

  config_->blackboard->set("path", path);

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }

  nav_msgs::msg::Path truncated_path;
  EXPECT_TRUE(config_->blackboard->get("truncated_path", truncated_path));

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_NE(path, truncated_path);
  ASSERT_GE(truncated_path.poses.size(), 1u);
  EXPECT_EQ(truncated_path.poses.size(), 3u);
  EXPECT_EQ(truncated_path.poses.front().pose.position.x, -0.5);
  EXPECT_EQ(truncated_path.poses.front().pose.position.y, 0.0);
  EXPECT_EQ(truncated_path.poses.back().pose.position.x, 1.5);
  EXPECT_EQ(truncated_path.poses.back().pose.position.y, 0.0);

  /////////////////////////////////////////
  // should match the first loop crossing
  config_->blackboard->set("pose", poseMsg(0.0, 0.0, M_PI / 2));

  tree_->haltTree();
  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }
  EXPECT_TRUE(config_->blackboard->get("truncated_path", truncated_path));

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_NE(path, truncated_path);
  ASSERT_GE(truncated_path.poses.size(), 1u);
  EXPECT_EQ(truncated_path.poses.size(), 2u);
  EXPECT_EQ(truncated_path.poses.front().pose.position.x, -0.3);
  EXPECT_EQ(truncated_path.poses.front().pose.position.y, 0.0);
  EXPECT_EQ(truncated_path.poses.back().pose.position.x, -0.5);
  EXPECT_EQ(truncated_path.poses.back().pose.position.y, 1.0);

  /////////////////////////////////////////
  // should match the last loop crossing
  config_->blackboard->set("pose", poseMsg(0.0, 0.0, -M_PI / 2));

  tree_->haltTree();
  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }
  EXPECT_TRUE(config_->blackboard->get("truncated_path", truncated_path));

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_NE(path, truncated_path);
  ASSERT_GE(truncated_path.poses.size(), 1u);
  EXPECT_EQ(truncated_path.poses.size(), 2u);
  EXPECT_EQ(truncated_path.poses.front().pose.position.x, 0.3);
  EXPECT_EQ(truncated_path.poses.front().pose.position.y, 0.0);
  EXPECT_EQ(truncated_path.poses.back().pose.position.x, 0.3);
  EXPECT_EQ(truncated_path.poses.back().pose.position.y, -1.0);

  SUCCEED();
}

TEST_F(TruncatePathLocalTestFixture, test_success_on_empty_path)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <TruncatePathLocal
            distance_forward="2.0"
            distance_backward="1.0"
            robot_frame="base_link"
            transform_tolerance="0.2"
            angular_distance_weight="0.2"
            pose="{pose}"
            input_path="{path}"
            output_path="{truncated_path}"
            max_robot_pose_search_dist="infinity"
          />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create path and set it on blackboard
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = "map";

  config_->blackboard->set("path", path);

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }
  nav_msgs::msg::Path truncated_path;
  EXPECT_TRUE(config_->blackboard->get("truncated_path", truncated_path));

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(path, truncated_path);
  SUCCEED();
}

TEST_F(TruncatePathLocalTestFixture, test_failure_on_no_pose)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <TruncatePathLocal
            distance_forward="2.0"
            distance_backward="1.0"
            transform_tolerance="0.2"
            angular_distance_weight="0.2"
            robot_frame="{robot_frame}"
            input_path="{path}"
            output_path="{truncated_path}"
            max_robot_pose_search_dist="infinity"
          />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create path and set it on blackboard
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = "map";

  config_->blackboard->set("path", path);

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }
  nav_msgs::msg::Path truncated_path;
  EXPECT_TRUE(config_->blackboard->get("truncated_path", truncated_path));

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
  SUCCEED();
}

TEST_F(TruncatePathLocalTestFixture, test_failure_on_invalid_robot_frame)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <TruncatePathLocal
            distance_forward="2.0"
            distance_backward="1.0"
            transform_tolerance="0.2"
            robot_frame="invalid_frame"
            angular_distance_weight="0.2"
            input_path="{path}"
            output_path="{truncated_path}"
            max_robot_pose_search_dist="infinity"
          />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new goal and set it on blackboard
  nav_msgs::msg::Path path = createLoopCrossingTestPath();
  EXPECT_EQ(path.poses.size(), 12u);

  config_->blackboard->set("path", path);

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }
  nav_msgs::msg::Path truncated_path;
  EXPECT_TRUE(config_->blackboard->get("truncated_path", truncated_path));

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
  SUCCEED();
}

TEST_F(TruncatePathLocalTestFixture, test_path_pruning)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <TruncatePathLocal
            distance_forward="2.0"
            distance_backward="1.0"
            robot_frame="base_link"
            transform_tolerance="0.2"
            angular_distance_weight="0.0"
            pose="{pose}"
            input_path="{path}"
            output_path="{truncated_path}"
            max_robot_pose_search_dist="3.0"
          />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create path and set it on blackboard
  nav_msgs::msg::Path path = createLoopCrossingTestPath();
  nav_msgs::msg::Path truncated_path;

  config_->blackboard->set("path", path);

  /////////////////////////////////////////
  // should match the first loop crossing
  config_->blackboard->set("pose", poseMsg(0.0, 0.0, 0.0));

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }
  EXPECT_TRUE(config_->blackboard->get("truncated_path", truncated_path));

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_NE(path, truncated_path);
  ASSERT_GE(truncated_path.poses.size(), 1u);
  EXPECT_EQ(truncated_path.poses.size(), 2u);
  EXPECT_EQ(truncated_path.poses.front().pose.position.x, -0.3);
  EXPECT_EQ(truncated_path.poses.front().pose.position.y, 0.0);
  EXPECT_EQ(truncated_path.poses.back().pose.position.x, -0.5);
  EXPECT_EQ(truncated_path.poses.back().pose.position.y, 1.0);

  /////////////////////////////////////////
  // move along the path to leave the first loop crossing behind
  config_->blackboard->set("pose", poseMsg(-1.5, 1.0, 0.0));
  // tick until node succeeds
  tree_->haltTree();
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }
  // this truncated_path is not interesting, let's proceed to the second loop crossing

  /////////////////////////////////////////
  // should match the second loop crossing
  config_->blackboard->set("pose", poseMsg(0.0, 0.0, 0.0));
  // tick until node succeeds
  tree_->haltTree();
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }
  EXPECT_TRUE(config_->blackboard->get("truncated_path", truncated_path));

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_NE(path, truncated_path);
  ASSERT_GE(truncated_path.poses.size(), 1u);
  EXPECT_EQ(truncated_path.poses.size(), 3u);
  EXPECT_EQ(truncated_path.poses.front().pose.position.x, -0.5);
  EXPECT_EQ(truncated_path.poses.front().pose.position.y, 0.0);
  EXPECT_EQ(truncated_path.poses.back().pose.position.x, 1.5);
  EXPECT_EQ(truncated_path.poses.back().pose.position.y, 0.0);

  /////////////////////////////////////////
  // move along the path to leave the second loop crossing behind
  config_->blackboard->set("pose", poseMsg(1.5, 1.0, 0.0));
  // tick until node succeeds
  tree_->haltTree();
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }
  // this truncated_path is not interesting, let's proceed to the last loop crossing

  /////////////////////////////////////////
  // should match the last loop crossing
  config_->blackboard->set("pose", poseMsg(0.0, 0.0, 0.0));
  // tick until node succeeds
  tree_->haltTree();
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }
  EXPECT_TRUE(config_->blackboard->get("truncated_path", truncated_path));

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_NE(path, truncated_path);
  ASSERT_GE(truncated_path.poses.size(), 1u);
  EXPECT_EQ(truncated_path.poses.size(), 2u);
  EXPECT_EQ(truncated_path.poses.front().pose.position.x, 0.3);
  EXPECT_EQ(truncated_path.poses.front().pose.position.y, 0.0);
  EXPECT_EQ(truncated_path.poses.back().pose.position.x, 0.3);
  EXPECT_EQ(truncated_path.poses.back().pose.position.y, -1.0);

  SUCCEED();
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
