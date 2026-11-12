// Copyright (c) 2026, Nisarg Panchal
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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "dwb_critics/goal_dist.hpp"
#include "dwb_critics/path_dist.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

using Cell = std::pair<unsigned int, unsigned int>;

class MapGridTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap", "/ns", false);
    costmap_ros_->configure();
    costmap_ = costmap_ros_->getCostmap();
  }

  template<class CriticT>
  std::shared_ptr<CriticT> makeCritic(bool obstacle_aware)
  {
    auto node = std::make_shared<nav2::LifecycleNode>("map_grid_tester");
    node->declare_parameter("ns.critic.obstacle_aware_propagation", obstacle_aware);
    auto critic = std::make_shared<CriticT>();
    critic->initialize(node, "critic", "ns", costmap_ros_);
    return critic;
  }

  geometry_msgs::msg::Pose pose(const Cell & cell)
  {
    geometry_msgs::msg::Pose pose;
    costmap_->mapToWorld(cell.first, cell.second, pose.position.x, pose.position.y);
    return pose;
  }

  nav_msgs::msg::Path plan(const std::vector<Cell> & cells)
  {
    nav_msgs::msg::Path path;
    for (const auto & cell : cells) {
      geometry_msgs::msg::PoseStamped stamped;
      stamped.pose = pose(cell);
      path.poses.push_back(stamped);
    }
    return path;
  }

  bool prepare(
    dwb_core::TrajectoryCritic & critic, const Cell & robot, const std::vector<Cell> & cells)
  {
    return critic.prepare(
      pose(robot), nav_2d_msgs::msg::Twist2D(), pose(cells.back()), plan(cells));
  }

  // Every cell of a column, as a planner would give it.
  std::vector<Cell> column(unsigned int x, unsigned int first_y, unsigned int last_y)
  {
    std::vector<Cell> cells;
    for (unsigned int y = first_y; y <= last_y; ++y) {
      cells.emplace_back(x, y);
    }
    return cells;
  }

  // A wall on column 10 from row 0 to row 14. The way around it is over row 15.
  void addWall()
  {
    for (unsigned int y = 0; y < 15; ++y) {
      costmap_->setCost(10, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  void surround(const Cell & cell)
  {
    for (unsigned int x = cell.first - 1; x <= cell.first + 1; ++x) {
      for (unsigned int y = cell.second - 1; y <= cell.second + 1; ++y) {
        if (x != cell.first || y != cell.second) {
          costmap_->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }

  double obstacleScore()
  {
    return static_cast<double>(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY());
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
};

TEST_F(MapGridTest, GoalDistGoesThroughObstaclesByDefault)
{
  addWall();
  auto critic = makeCritic<dwb_critics::GoalDistCritic>(false);
  ASSERT_TRUE(prepare(*critic, {5, 5}, {{5, 5}, {15, 5}}));
  EXPECT_EQ(critic->getScore(5, 5), 10.0);
  EXPECT_EQ(critic->getScore(10, 5), 5.0);
}

TEST_F(MapGridTest, GoalDistGoesAroundAWall)
{
  addWall();
  auto critic = makeCritic<dwb_critics::GoalDistCritic>(true);
  // The straight plan is blocked by the wall, so the local goal is past it.
  ASSERT_TRUE(prepare(*critic, {5, 5}, {{5, 5}, {15, 5}}));
  EXPECT_EQ(critic->getScore(15, 5), 0.0);
  EXPECT_EQ(critic->getScore(5, 5), 30.0);
  EXPECT_EQ(critic->getScore(10, 5), obstacleScore());
}

TEST_F(MapGridTest, GoalDistCountsTheStepsAroundAPocket)
{
  // A pocket open to the right, with the goal outside and to its left.
  for (unsigned int y = 10; y <= 20; ++y) {
    costmap_->setCost(10, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  for (unsigned int x = 10; x <= 20; ++x) {
    costmap_->setCost(x, 10, nav2_costmap_2d::LETHAL_OBSTACLE);
    costmap_->setCost(x, 20, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  auto critic = makeCritic<dwb_critics::GoalDistCritic>(true);
  ASSERT_TRUE(prepare(*critic, {11, 15}, {{11, 15}, {9, 15}}));
  // Out through column 21 and back over row 9 or row 21.
  EXPECT_EQ(critic->getScore(11, 15), 10.0 + 6.0 + 12.0 + 6.0);
}

TEST_F(MapGridTest, GoalDistFallsBackWhenTheGoalIsBlocked)
{
  costmap_->setCost(15, 5, nav2_costmap_2d::LETHAL_OBSTACLE);
  auto critic = makeCritic<dwb_critics::GoalDistCritic>(true);
  ASSERT_TRUE(prepare(*critic, {5, 5}, {{5, 5}, {15, 5}}));
  EXPECT_EQ(critic->getScore(15, 5), 0.0);
  EXPECT_EQ(critic->getScore(5, 5), 10.0);
}

TEST_F(MapGridTest, GoalDistFallsBackWhenTheRobotCellIsUnreachable)
{
  surround({5, 5});
  auto critic = makeCritic<dwb_critics::GoalDistCritic>(true);
  ASSERT_TRUE(prepare(*critic, {5, 5}, {{5, 5}, {15, 5}}));
  EXPECT_EQ(critic->getScore(15, 5), 0.0);
  EXPECT_EQ(critic->getScore(5, 5), 10.0);
  EXPECT_EQ(critic->getScore(6, 5), 9.0);
}

TEST_F(MapGridTest, GoalDistFallsBackWhenTheRobotIsInAnObstacle)
{
  costmap_->setCost(5, 5, nav2_costmap_2d::LETHAL_OBSTACLE);
  auto critic = makeCritic<dwb_critics::GoalDistCritic>(true);
  ASSERT_TRUE(prepare(*critic, {5, 5}, {{5, 5}, {15, 5}}));
  EXPECT_EQ(critic->getScore(5, 5), 10.0);
}

TEST_F(MapGridTest, GoalDistGoesThroughInscribedAndUnknownCells)
{
  for (unsigned int y = 0; y < 15; ++y) {
    costmap_->setCost(
      10, y, y % 2 ? nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE :
      nav2_costmap_2d::NO_INFORMATION);
  }
  auto critic = makeCritic<dwb_critics::GoalDistCritic>(true);
  ASSERT_TRUE(prepare(*critic, {5, 5}, {{5, 5}, {15, 5}}));
  EXPECT_EQ(critic->getScore(5, 5), 10.0);
  EXPECT_EQ(critic->getScore(10, 5), 5.0);
  EXPECT_EQ(critic->getScore(10, 6), 6.0);
}

TEST_F(MapGridTest, GoalDistFallsBackWhenEverySeedIsBlocked)
{
  for (unsigned int x = 12; x <= 15; ++x) {
    costmap_->setCost(x, 5, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  auto critic = makeCritic<dwb_critics::GoalDistCritic>(true);
  ASSERT_TRUE(prepare(*critic, {5, 5}, {{12, 5}, {15, 5}}));
  EXPECT_EQ(critic->getScore(15, 5), 0.0);
  EXPECT_EQ(critic->getScore(5, 5), 10.0);
}

TEST_F(MapGridTest, GoalDistFailsWhenThePlanIsOffTheCostmap)
{
  auto critic = makeCritic<dwb_critics::GoalDistCritic>(true);
  nav_msgs::msg::Path off_map;
  off_map.poses.resize(1);
  off_map.poses[0].pose.position.x = -1.0;
  EXPECT_FALSE(
    critic->prepare(
      pose({5, 5}), nav_2d_msgs::msg::Twist2D(), off_map.poses[0].pose, off_map));
}

TEST_F(MapGridTest, PathDistGoesThroughObstaclesByDefault)
{
  addWall();
  auto critic = makeCritic<dwb_critics::PathDistCritic>(false);
  ASSERT_TRUE(prepare(*critic, {5, 5}, column(15, 5, 10)));
  EXPECT_EQ(critic->getScore(5, 5), 10.0);
}

TEST_F(MapGridTest, PathDistGoesAroundAWall)
{
  addWall();
  auto critic = makeCritic<dwb_critics::PathDistCritic>(true);
  ASSERT_TRUE(prepare(*critic, {5, 5}, column(15, 5, 10)));
  EXPECT_EQ(critic->getScore(15, 7), 0.0);
  EXPECT_EQ(critic->getScore(5, 5), 25.0);
}

TEST_F(MapGridTest, PathDistSkipsBlockedSeeds)
{
  costmap_->setCost(15, 7, nav2_costmap_2d::LETHAL_OBSTACLE);
  auto critic = makeCritic<dwb_critics::PathDistCritic>(true);
  ASSERT_TRUE(prepare(*critic, {5, 5}, column(15, 5, 10)));
  EXPECT_EQ(critic->getScore(15, 6), 0.0);
  EXPECT_EQ(critic->getScore(15, 7), obstacleScore());
}

TEST_F(MapGridTest, PathDistFallsBackWhenTheRobotCellIsUnreachable)
{
  surround({5, 5});
  auto critic = makeCritic<dwb_critics::PathDistCritic>(true);
  ASSERT_TRUE(prepare(*critic, {5, 5}, column(15, 5, 10)));
  EXPECT_EQ(critic->getScore(5, 5), 10.0);
  EXPECT_EQ(critic->getScore(15, 7), 0.0);
}

TEST_F(MapGridTest, PathDistFallsBackWhenEverySeedIsBlocked)
{
  for (unsigned int y = 5; y <= 10; ++y) {
    costmap_->setCost(15, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  auto critic = makeCritic<dwb_critics::PathDistCritic>(true);
  ASSERT_TRUE(prepare(*critic, {5, 5}, column(15, 5, 10)));
  EXPECT_EQ(critic->getScore(15, 7), 0.0);
  EXPECT_EQ(critic->getScore(5, 5), 10.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
