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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rclcpp/utilities.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/concatenate_paths_action.hpp"
#include "utils/test_behavior_tree_fixture.hpp"


class ConcatenatePathsTestFixture : public ::testing::Test
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
      "transform_manager",
      transform_handler_->getTfManager());

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::ConcatenatePaths>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::ConcatenatePaths>(
      "ConcatenatePaths", builder);
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

  static nav_msgs::msg::Path makePath(const std::string & frame, double x0, int count)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame;
    path.header.stamp = node_->now();
    for (int i = 0; i < count; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x0 + i;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  static BT::NodeStatus runTree(const std::string & xml_txt)
  {
    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
    while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
      tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
    {
      tree_->rootNode()->executeTick();
    }
    return tree_->rootNode()->status();
  }

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
  static std::shared_ptr<nav2_behavior_tree::TransformHandler> transform_handler_;
};

nav2::LifecycleNode::SharedPtr ConcatenatePathsTestFixture::node_ = nullptr;

BT::NodeConfiguration * ConcatenatePathsTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ConcatenatePathsTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ConcatenatePathsTestFixture::tree_ = nullptr;
std::shared_ptr<nav2_behavior_tree::TransformHandler>
ConcatenatePathsTestFixture::transform_handler_ = nullptr;

TEST_F(ConcatenatePathsTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <ConcatenatePaths input_path1="{path1}" input_path2="{path2}" output_path="{concat_path}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new goal and set it on blackboard
  nav_msgs::msg::Path path1, path2;
  path1.header.stamp = node_->now();
  path2.header.stamp = node_->now();

  int i = 0;
  int j = 3;
  for (int x = 0; x != 3; x++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    path1.poses.push_back(pose);
    pose.pose.position.x = j;
    path2.poses.push_back(pose);
    i++;
    j++;
  }

  config_->blackboard->set("path1", path1);
  config_->blackboard->set("path2", path2);

  // tick until node finishes
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  nav_msgs::msg::Path concat_path;
  EXPECT_TRUE(config_->blackboard->get("concat_path", concat_path));

  EXPECT_EQ(concat_path.poses.size(), 6u);
  for (size_t x = 0; x < concat_path.poses.size(); ++x) {
    EXPECT_EQ(concat_path.poses[x].pose.position.x, static_cast<double>(x));
  }

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  config_->blackboard->set("path1", nav_msgs::msg::Path());
  config_->blackboard->set("path2", nav_msgs::msg::Path());
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
}

TEST_F(ConcatenatePathsTestFixture, test_same_frame)
{
  config_->blackboard->set("path1", makePath("map", 0.0, 2));
  config_->blackboard->set("path2", makePath("map", 2.0, 2));

  EXPECT_EQ(
    runTree(
      R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <ConcatenatePaths input_path1="{path1}" input_path2="{path2}"
            output_path="{concat_path}"/>
        </BehaviorTree>
      </root>)"),
    BT::NodeStatus::SUCCESS);

  nav_msgs::msg::Path concat_path;
  EXPECT_TRUE(config_->blackboard->get("concat_path", concat_path));
  EXPECT_EQ(concat_path.header.frame_id, "map");
  ASSERT_EQ(concat_path.poses.size(), 4u);
  for (size_t i = 0; i < concat_path.poses.size(); ++i) {
    EXPECT_EQ(concat_path.poses[i].pose.position.x, static_cast<double>(i));
  }
}

TEST_F(ConcatenatePathsTestFixture, test_different_frames_fails_by_default)
{
  config_->blackboard->set("path1", makePath("map", 0.0, 2));
  config_->blackboard->set("path2", makePath("odom", 0.0, 2));

  EXPECT_EQ(
    runTree(
      R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <ConcatenatePaths input_path1="{path1}" input_path2="{path2}"
            output_path="{concat_path}"/>
        </BehaviorTree>
      </root>)"),
    BT::NodeStatus::FAILURE);
}

TEST_F(ConcatenatePathsTestFixture, test_different_frames_transformed)
{
  // odom is offset by 10 m in x from map
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.transform.translation.x = 10.0;
  transform.transform.rotation.w = 1.0;
  transform_handler_->getTfManager()->getRawBuffer()->setTransform(transform, "unit_test", true);

  config_->blackboard->set("path1", makePath("map", 0.0, 2));
  config_->blackboard->set("path2", makePath("odom", 0.0, 2));

  EXPECT_EQ(
    runTree(
      R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <ConcatenatePaths input_path1="{path1}" input_path2="{path2}" output_path="{concat_path}"
            allow_frame_transform="true"/>
        </BehaviorTree>
      </root>)"),
    BT::NodeStatus::SUCCESS);

  nav_msgs::msg::Path concat_path;
  EXPECT_TRUE(config_->blackboard->get("concat_path", concat_path));
  EXPECT_EQ(concat_path.header.frame_id, "map");
  ASSERT_EQ(concat_path.poses.size(), 4u);
  EXPECT_DOUBLE_EQ(concat_path.poses[0].pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(concat_path.poses[1].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(concat_path.poses[2].pose.position.x, 10.0);
  EXPECT_DOUBLE_EQ(concat_path.poses[3].pose.position.x, 11.0);
  for (const auto & pose : concat_path.poses) {
    EXPECT_EQ(pose.header.frame_id, "map");
  }
}

TEST_F(ConcatenatePathsTestFixture, test_transform_unavailable_fails)
{
  config_->blackboard->set("path1", makePath("map", 0.0, 2));
  config_->blackboard->set("path2", makePath("unknown_frame", 0.0, 2));

  EXPECT_EQ(
    runTree(
      R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <ConcatenatePaths input_path1="{path1}" input_path2="{path2}" output_path="{concat_path}"
            allow_frame_transform="true"/>
        </BehaviorTree>
      </root>)"),
    BT::NodeStatus::FAILURE);
}

TEST_F(ConcatenatePathsTestFixture, test_empty_first_path_different_frame)
{
  // An empty first path still sets the output frame, so its frame is checked too
  config_->blackboard->set("path1", makePath("map", 0.0, 0));
  config_->blackboard->set("path2", makePath("odom", 0.0, 2));

  EXPECT_EQ(
    runTree(
      R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <ConcatenatePaths input_path1="{path1}" input_path2="{path2}"
            output_path="{concat_path}"/>
        </BehaviorTree>
      </root>)"),
    BT::NodeStatus::FAILURE);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.transform.translation.x = 10.0;
  transform.transform.rotation.w = 1.0;
  transform_handler_->getTfManager()->getRawBuffer()->setTransform(transform, "unit_test", true);

  EXPECT_EQ(
    runTree(
      R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <ConcatenatePaths input_path1="{path1}" input_path2="{path2}" output_path="{concat_path}"
            allow_frame_transform="true"/>
        </BehaviorTree>
      </root>)"),
    BT::NodeStatus::SUCCESS);

  nav_msgs::msg::Path concat_path;
  EXPECT_TRUE(config_->blackboard->get("concat_path", concat_path));
  EXPECT_EQ(concat_path.header.frame_id, "map");
  ASSERT_EQ(concat_path.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(concat_path.poses[0].pose.position.x, 10.0);
  EXPECT_DOUBLE_EQ(concat_path.poses[1].pose.position.x, 11.0);
  for (const auto & pose : concat_path.poses) {
    EXPECT_EQ(pose.header.frame_id, "map");
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
