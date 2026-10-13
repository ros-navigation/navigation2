// Copyright (c) 2026 Origin
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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/classified_pose.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/action/path_class_controller_selector_node.hpp"

using nav2_msgs::msg::ClassifiedPose;

static constexpr uint16_t FREE = ClassifiedPose::FREE_SPACE;
static constexpr uint16_t CONFINED = ClassifiedPose::CONFINED_SPACE;
static constexpr double kSpacing = 0.1;
static const char kPorts[] =
  R"(class_controller_map="1:FollowPath;2:ConfinedFollowPath" lookahead_arc_dist="1.0")";

class PathClassControllerSelectorTestFixture
  : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  static void SetUpTestCase()
  {
    BehaviorTreeTestFixture::SetUpTestCase();
    factory_->registerNodeType<nav2_behavior_tree::PathClassControllerSelector>(
      "PathClassControllerSelector");
  }

  void SetUp() override
  {
    config_->blackboard->set<nav_msgs::msg::Path>("path", nav_msgs::msg::Path());
    config_->blackboard->set<std::vector<ClassifiedPose>>("classified_path", {});
    config_->blackboard->set<std::string>("selected_controller", "");
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  void createTree(const std::string & attributes)
  {
    const std::string xml_txt =
      R"(<root main_tree_to_execute="MainTree">
           <BehaviorTree ID="MainTree">
             <PathClassControllerSelector path="{path}" classified_path="{classified_path}"
               selected_controller="{selected_controller}" )" + attributes +
      R"(/>
           </BehaviorTree>
         </root>)";
    tree_ = std::make_shared<BT::Tree>(
      factory_->createTreeFromText(xml_txt, config_->blackboard));
  }

  static nav_msgs::msg::Path makePath(const std::vector<std::array<double, 2>> & points)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (const auto & point : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = point[0];
      pose.pose.position.y = point[1];
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  static nav_msgs::msg::Path makeStraightPath(double start_x, size_t num_poses)
  {
    std::vector<std::array<double, 2>> points;
    for (size_t i = 0; i < num_poses; ++i) {
      points.push_back({start_x + i * kSpacing, 0.0});
    }
    return makePath(points);
  }

  // FREE everywhere except CONFINED on poses [begin, end)
  static std::vector<uint16_t> makeClasses(size_t num_poses, size_t begin, size_t end)
  {
    std::vector<uint16_t> classes(num_poses, FREE);
    std::fill(classes.begin() + begin, classes.begin() + end, CONFINED);
    return classes;
  }

  void setPath(const nav_msgs::msg::Path & path, const std::vector<uint16_t> & classes)
  {
    std::vector<ClassifiedPose> classified_path(classes.size());
    for (size_t i = 0; i < classes.size(); ++i) {
      classified_path[i].pose = path.poses[i];
      classified_path[i].class_type = classes[i];
    }
    config_->blackboard->set<nav_msgs::msg::Path>("path", path);
    config_->blackboard->set<std::vector<ClassifiedPose>>("classified_path", classified_path);
  }

  bool setRobotPose(double x, double y)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation.w = 1.0;
    transform_handler_->updateRobotPose(pose);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    geometry_msgs::msg::PoseStamped current;
    while (std::chrono::steady_clock::now() < deadline) {
      if (nav2_util::getCurrentPose(current, *transform_handler_->getBuffer()) &&
        std::hypot(current.pose.position.x - x, current.pose.position.y - y) < 1e-6)
      {
        return true;
      }
    }
    return false;
  }

  std::string tickAndGetController()
  {
    EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
    return config_->blackboard->get<std::string>("selected_controller");
  }

  std::shared_ptr<BT::Tree> tree_;
};

TEST_F(PathClassControllerSelectorTestFixture, invalid_map_throws)
{
  for (const std::string map : {"1FollowPath", ":FollowPath", "1:", "abc:FollowPath"}) {
    EXPECT_THROW(
      createTree(R"(class_controller_map=")" + map + R"(")"), std::runtime_error) << map;
  }
}

TEST_F(PathClassControllerSelectorTestFixture, falls_back_to_default)
{
  createTree(std::string(kPorts) + R"( default_controller="Default")");
  EXPECT_EQ(tickAndGetController(), "Default");

  ASSERT_TRUE(setRobotPose(0.02, 0.0));
  setPath(makeStraightPath(0.0, 30), std::vector<uint16_t>(30, 7));
  EXPECT_EQ(tickAndGetController(), "Default");
}

TEST_F(PathClassControllerSelectorTestFixture, fails_without_default)
{
  createTree(kPorts);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(PathClassControllerSelectorTestFixture, keeps_selection_when_paths_mismatch)
{
  createTree(std::string(kPorts) + R"( default_controller="Default")");
  ASSERT_TRUE(setRobotPose(0.02, 0.0));
  const auto path = makeStraightPath(0.0, 30);

  setPath(path, makeClasses(30, 0, 30));
  EXPECT_EQ(tickAndGetController(), "ConfinedFollowPath");

  // Path replanned but not yet classified
  setPath(path, makeClasses(10, 0, 0));
  EXPECT_EQ(tickAndGetController(), "ConfinedFollowPath");
}

TEST_F(PathClassControllerSelectorTestFixture, selects_class_at_lookahead)
{
  createTree(kPorts);
  // CONFINED on x in [2.0, 3.0) and on the last pose at x = 4.9
  auto classes = makeClasses(50, 20, 30);
  classes.back() = CONFINED;
  setPath(makeStraightPath(0.0, 50), classes);

  ASSERT_TRUE(setRobotPose(0.52, 0.0));
  EXPECT_EQ(tickAndGetController(), "FollowPath");

  // Lookahead enters the confined segment before the robot does
  ASSERT_TRUE(setRobotPose(1.22, 0.0));
  EXPECT_EQ(tickAndGetController(), "ConfinedFollowPath");

  // Lookahead leaves the confined segment while the robot is still inside it
  ASSERT_TRUE(setRobotPose(2.22, 0.0));
  EXPECT_EQ(tickAndGetController(), "FollowPath");

  // Lookahead runs past the path end, so the last pose is used
  ASSERT_TRUE(setRobotPose(3.12, 0.0));
  EXPECT_EQ(tickAndGetController(), "FollowPath");
  ASSERT_TRUE(setRobotPose(4.02, 0.0));
  EXPECT_EQ(tickAndGetController(), "ConfinedFollowPath");
}

TEST_F(PathClassControllerSelectorTestFixture, ignores_u_turn_return_leg)
{
  createTree(kPorts);
  // Out along y = 0 as FREE, back along y = 0.2 as CONFINED
  std::vector<std::array<double, 2>> points;
  for (size_t i = 0; i < 20; ++i) {
    points.push_back({i * kSpacing, 0.0});
  }
  for (size_t i = 0; i < 20; ++i) {
    points.push_back({(19 - i) * kSpacing, 0.2});
  }
  setPath(makePath(points), makeClasses(40, 20, 40));

  // Closer to the return leg, which is outside the search window
  ASSERT_TRUE(setRobotPose(0.52, 0.15));
  EXPECT_EQ(tickAndGetController(), "FollowPath");
}

TEST_F(PathClassControllerSelectorTestFixture, resets_cursor_on_new_path)
{
  createTree(kPorts);
  setPath(makeStraightPath(0.0, 50), makeClasses(50, 0, 0));
  for (double x : {0.52, 1.22, 2.02}) {
    ASSERT_TRUE(setRobotPose(x, 0.0));
    EXPECT_EQ(tickAndGetController(), "FollowPath");
  }

  // New path starts at the robot and is CONFINED for its first 1.5 m. A stale
  // cursor at index 20 would land in the FREE part
  setPath(makeStraightPath(2.0, 50), makeClasses(50, 0, 15));
  EXPECT_EQ(tickAndGetController(), "ConfinedFollowPath");
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
