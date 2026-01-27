// Copyright (c) 2026 Jakub Chudziński
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
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2_ros/buffer.hpp"

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/is_goal_nearby_condition.hpp"

using namespace std::chrono_literals;

class IsGoalNearbyConditionTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp() override
  {
    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::IsGoalNearbyCondition>(name, config);
      };

    try {
      factory_->registerBuilder<nav2_behavior_tree::IsGoalNearbyCondition>("IsGoalNearby", builder);
    } catch (BT::BehaviorTreeException const &) {
    }
    tf_buffer_ = config_->blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  void setRobotPoseInMap(double x, double y)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = node_->now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    tf_buffer_->setTransform(tf, "unit_test", true);
  }

  static nav_msgs::msg::Path makePathInMap(
    const std::vector<std::pair<double, double>> & pts)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";

    for (const auto & [x, y] : pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.pose.position.x = x;
      ps.pose.position.y = y;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    return path;
  }

  static nav_msgs::msg::Path makeStraightPathInMap(
    const std::vector<double> & xs)
  {
    std::vector<std::pair<double, double>> pts;
    pts.reserve(xs.size());

    for (double x : xs) {
      pts.emplace_back(x, 0.0);
    }

    return makePathInMap(pts);
  }

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  static std::shared_ptr<BT::Tree> tree_;
};

std::shared_ptr<BT::Tree> IsGoalNearbyConditionTestFixture::tree_ = nullptr;

TEST_F(IsGoalNearbyConditionTestFixture, empty_path_returns_failure)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsGoalNearby
            path="{path}"
            proximity_threshold="1.0"
            max_robot_pose_search_dist="-1"
          />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  nav_msgs::msg::Path empty;
  empty.header.frame_id = "map";
  config_->blackboard->set("path", empty);

  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
}

TEST_F(IsGoalNearbyConditionTestFixture, far_from_goal_returns_failure)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsGoalNearby
            path="{path}"
            proximity_threshold="0.5"
            max_robot_pose_search_dist="-1"
          />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  auto path = makeStraightPathInMap({0.0, 1.0, 2.0});
  config_->blackboard->set("path", path);

  setRobotPoseInMap(0.1, 0.0);

  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
}

TEST_F(IsGoalNearbyConditionTestFixture, near_goal_returns_success)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsGoalNearby
            path="{path}"
            proximity_threshold="0.5"
            max_robot_pose_search_dist="-1"
          />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  auto path = makeStraightPathInMap({0.0, 1.0, 2.0});
  config_->blackboard->set("path", path);

  setRobotPoseInMap(1.8, 0.0);

  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsGoalNearbyConditionTestFixture,
  pruning_bounded_search_ignores_far_ahead_close_point)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsGoalNearby
            path="{path}"
            proximity_threshold="0.5"
            max_robot_pose_search_dist="{max_robot_pose_search_dist}"
          />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  auto path = makeStraightPathInMap({
    0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0});
  config_->blackboard->set("path", path);

  setRobotPoseInMap(5.0, 0.0);

  config_->blackboard->set("max_robot_pose_search_dist", 2.0);
  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);

  tree_->haltTree();

  config_->blackboard->set("max_robot_pose_search_dist", -1.0);
  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);

  tree_->haltTree();

  setRobotPoseInMap(9.8, 0.0);
  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsGoalNearbyConditionTestFixture, path_pruning_coverage)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsGoalNearby
            path="{path}"
            proximity_threshold="0.5"
            max_robot_pose_search_dist="2.5"
          />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  auto path = makeStraightPathInMap({0.0, 1.0, 2.0, 3.0, 4.0, 5.0});
  config_->blackboard->set("path", path);

  setRobotPoseInMap(1.0, 0.0);
  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
}

TEST_F(IsGoalNearbyConditionTestFixture, invalid_robot_frame_returns_failure)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsGoalNearby
            path="{path}"
            proximity_threshold="0.5"
            robot_base_frame="nonexistent_robot_frame"
          />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  auto path = makeStraightPathInMap({0.0, 1.0});
  config_->blackboard->set("path", path);

  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
}

TEST_F(IsGoalNearbyConditionTestFixture, invalid_path_frame_returns_failure)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsGoalNearby
            path="{path}"
            proximity_threshold="0.5"
          />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // Create path in non-existent frame to trigger transform error
  nav_msgs::msg::Path path;
  path.header.frame_id = "nonexistent_frame";
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = "nonexistent_frame";
  ps.pose.position.x = 1.0;
  ps.pose.orientation.w = 1.0;
  path.poses.push_back(ps);

  config_->blackboard->set("path", path);
  setRobotPoseInMap(0.0, 0.0);

  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  const bool all_successful = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return all_successful;
}
