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
#include "dwb_critics/path_hug.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "dwb_msgs/msg/trajectory2_d.hpp"

static constexpr double default_forward_point_distance = 0.325;

class PathHugCriticTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("test_node");
    node_->configure();
    node_->activate();

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap", "",
      false);
    costmap_ros_->configure();

    std::string name = "PathHugCritic";
    critic_ = std::make_shared<dwb_critics::PathHugCritic>();
    critic_->initialize(node_, name, "", costmap_ros_);
    critic_->onInit();
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
      pose.orientation.w = 0.0;
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
  node_->declare_parameter("PathHugCritic.forward_point_distance", default_forward_point_distance);
  EXPECT_EQ(
    node_->get_parameter("PathHugCritic.forward_point_distance").as_double(),
    default_forward_point_distance);
}

TEST_F(PathHugCriticTest, HandlesEmptyTrajectory)
{
  nav_msgs::msg::Path path = makePath({{0.0, 0.0}, {1.0, 1.0}});
  critic_->prepare(makePose(0, 0), nav_2d_msgs::msg::Twist2D(), makePose(1, 1), path);

  dwb_msgs::msg::Trajectory2D traj;
  EXPECT_DOUBLE_EQ(critic_->scoreTrajectory(traj), 0.0);
}

TEST_F(PathHugCriticTest, HandlesEmptyGlobalPath)
{
  nav_msgs::msg::Path path;
  critic_->prepare(makePose(0, 0), nav_2d_msgs::msg::Twist2D(), makePose(1, 1), path);

  dwb_msgs::msg::Trajectory2D traj = makeTrajectory({{0.0, 0.0}, {1.0, 1.0}});
  EXPECT_DOUBLE_EQ(critic_->scoreTrajectory(traj), 0.0);
}

TEST_F(PathHugCriticTest, ScoresTrajectoryNearPath)
{
  nav_msgs::msg::Path path = makePath({{0.0, 0.0}, {1.0, 1.0}});
  critic_->prepare(makePose(0, 0), nav_2d_msgs::msg::Twist2D(), makePose(1, 1), path);

  dwb_msgs::msg::Trajectory2D traj = makeTrajectory({{0.0, 0.0}, {0.5, 0.5}, {1.0, 1.0}});
  double score = critic_->scoreTrajectory(traj);
  EXPECT_GE(score, 0.0);
}

TEST_F(PathHugCriticTest, ScoresTrajectoryFarFromPath)
{
  nav_msgs::msg::Path path = makePath({{0.0, 0.0}, {1.0, 1.0}});
  critic_->prepare(makePose(0, 0), nav_2d_msgs::msg::Twist2D(), makePose(1, 1), path);

  dwb_msgs::msg::Trajectory2D traj = makeTrajectory({{5.0, 5.0}, {6.0, 6.0}});
  double score = critic_->scoreTrajectory(traj);
  EXPECT_GT(score, 0.0);
}

TEST_F(PathHugCriticTest, CustomParameterValues)
{
  auto custom_node = std::make_shared<nav2::LifecycleNode>("custom_test_node");
  custom_node->configure();
  custom_node->activate();

  double custom_distance = 1.0;
  std::string name = "PathHugCritic";

  nav2::declare_parameter_if_not_declared(
    custom_node, name + ".forward_point_distance",
    rclcpp::ParameterValue(custom_distance));

  auto custom_critic = std::make_shared<dwb_critics::PathHugCritic>();
  auto custom_costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("custom_costmap", "",
    false);
  custom_costmap->configure();

  custom_critic->initialize(custom_node, name, "", custom_costmap);
  custom_critic->onInit();

  EXPECT_EQ(
    custom_node->get_parameter(name + ".forward_point_distance").as_double(),
    custom_distance);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
