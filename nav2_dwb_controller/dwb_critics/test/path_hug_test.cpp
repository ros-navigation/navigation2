// Copyright (c) 2025, Berkan Tali
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
#include <vector>
#include <utility>

#include "dwb_critics/path_hug.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "dwb_msgs/msg/trajectory2_d.hpp"

class PathHugCriticTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("test_node");
    node_->configure();
    node_->activate();

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "test_global_costmap", "", false);
    costmap_ros_->configure();

    critic_ = std::make_shared<dwb_critics::PathHugCritic>();
    critic_->initialize(node_, "PathHugCritic", "FollowPath", costmap_ros_);
  }

  geometry_msgs::msg::Pose makePose(double x, double y)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation.w = 1.0;
    return pose;
  }

  nav_msgs::msg::Path makePath(std::vector<std::pair<double, double>> points)
  {
    nav_msgs::msg::Path path;
    for (const auto & pt : points) {
      geometry_msgs::msg::PoseStamped ps;
      ps.pose = makePose(pt.first, pt.second);
      path.poses.push_back(ps);
    }
    return path;
  }

  dwb_msgs::msg::Trajectory2D makeTrajectory(std::vector<std::pair<double, double>> points)
  {
    dwb_msgs::msg::Trajectory2D traj;
    for (const auto & pt : points) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = pt.first;
      pose.position.y = pt.second;
      pose.orientation.w = 1.0;
      traj.poses.push_back(pose);
    }
    return traj;
  }

  std::shared_ptr<nav2::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<dwb_critics::PathHugCritic> critic_;
};

TEST_F(PathHugCriticTest, DefaultParameterInitialization)
{
  EXPECT_DOUBLE_EQ(
    node_->get_parameter("FollowPath.PathHugCritic.scale").as_double(), 1.0);
  EXPECT_DOUBLE_EQ(
    node_->get_parameter("FollowPath.PathHugCritic.search_window").as_double(), 0.15);
  EXPECT_DOUBLE_EQ(
    node_->get_parameter("FollowPath.PathHugCritic.max_allowed_distance").as_double(), 0.05);
  EXPECT_DOUBLE_EQ(
    node_->get_parameter("FollowPath.PathHugCritic.critical_cost").as_double(), 100.0);
}

TEST_F(PathHugCriticTest, HandlesEmptyTrajectory)
{
  nav_msgs::msg::Path path = makePath({{0.0, 0.0}, {1.0, 0.0}});
  critic_->prepare(makePose(0, 0), nav_2d_msgs::msg::Twist2D(), makePose(1, 0), path);

  EXPECT_DOUBLE_EQ(critic_->scoreTrajectory(dwb_msgs::msg::Trajectory2D()), 0.0);
}

TEST_F(PathHugCriticTest, HandlesEmptyGlobalPath)
{
  critic_->prepare(
    makePose(0, 0), nav_2d_msgs::msg::Twist2D(), makePose(1, 0), nav_msgs::msg::Path());

  EXPECT_DOUBLE_EQ(critic_->scoreTrajectory(makeTrajectory({{0.0, 0.0}, {1.0, 0.0}})), 0.0);
}

TEST_F(PathHugCriticTest, ScoresZeroOnPath)
{
  nav_msgs::msg::Path path = makePath({{0.0, 0.0}, {0.5, 0.0}});
  critic_->prepare(makePose(0, 0), nav_2d_msgs::msg::Twist2D(), makePose(0.5, 0), path);

  EXPECT_DOUBLE_EQ(
    critic_->scoreTrajectory(makeTrajectory({{0.0, 0.0}, {0.1, 0.0}, {0.5, 0.0}})), 0.0);
}

TEST_F(PathHugCriticTest, FartherTrajectoryScoresHigher)
{
  nav_msgs::msg::Path path = makePath({{0.0, 0.0}, {2.0, 0.0}});
  critic_->prepare(makePose(0, 0), nav_2d_msgs::msg::Twist2D(), makePose(2, 0), path);

  // Both trajectories exceed max_allowed_distance (0.05 m), so both incur critical_cost.
  // The farther one must score strictly higher.
  double score_near = critic_->scoreTrajectory(makeTrajectory({{1.0, 0.1}}));
  double score_far = critic_->scoreTrajectory(makeTrajectory({{1.0, 0.5}}));

  EXPECT_GT(score_far, score_near);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
