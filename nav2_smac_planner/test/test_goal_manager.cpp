// Copyright (c) 2023 Open Navigation LLC
// Copyright (c) 2024 Stevedan Ogochukwu Omodolor Omodia
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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_smac_planner/goal_manager.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_smac_planner/collision_checker.hpp"

using namespace nav2_smac_planner; // NOLINT

using GoalManagerHybrid = GoalManager<NodeHybrid>;
using NodePtr = NodeHybrid *;
using NodeVector = GoalManagerHybrid::NodeVector;
using CoordinateVector = GoalManagerHybrid::CoordinateVector;
using GoalStateVector = GoalManagerHybrid::GoalStateVector;

TEST(GoalManagerTest, test_goal_manager)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");

  auto costmapA = new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);

  // Create an island of lethal cost in the middle
  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmapA->setCost(i, j, 254);
    }
  }

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmapA;

  auto checker = std::make_unique<nav2_smac_planner::GridCollisionChecker>(
    costmap_ros, 72, node);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  GoalManagerHybrid goal_manager;
  float tolerance = 20.0f;
  bool allow_unknow = false;

  EXPECT_TRUE(goal_manager.goalsIsEmpty());

  // Create two valid goals
  NodePtr pose_a = new NodeHybrid(48);
  NodePtr pose_b = new NodeHybrid(49);
  pose_a->setPose(NodeHybrid::Coordinates(0, 0, 0));
  pose_b->setPose(NodeHybrid::Coordinates(0, 0, 10));

  GoalStateVector goals_state = {
    {pose_a, true},
    {pose_b, true}
  };

  goal_manager.setGoalStates(goals_state);
  EXPECT_FALSE(goal_manager.goalsIsEmpty());
  EXPECT_EQ(goal_manager.getGoalsState().size(), 2u);
  EXPECT_TRUE(goal_manager.getGoalsSet().empty());
  EXPECT_TRUE(goal_manager.getGoalsCoordinates().empty());

  goal_manager.removeInvalidGoals(tolerance, checker.get(), allow_unknow);

  EXPECT_EQ(goal_manager.getGoalsSet().size(), 2);
  EXPECT_EQ(goal_manager.getGoalsCoordinates().size(), 2);
  for (const auto & goal_state : goal_manager.getGoalsState()) {
    EXPECT_TRUE(goal_state.is_valid);
  }

  // Test is goal
  EXPECT_TRUE(goal_manager.isGoal(goals_state[0].goal));
  EXPECT_TRUE(goal_manager.isGoal(goals_state[1].goal));

  // Re-populate and validate reset state
  goal_manager.setGoalStates(goals_state);
  EXPECT_EQ(goal_manager.getGoalsSet().size(), 0);
  EXPECT_EQ(goal_manager.getGoalsCoordinates().size(), 0);

  // Add invalid goal
  NodeHybrid pose_c(50);
  pose_c.setPose(NodeHybrid::Coordinates(50, 50, 0));  // inside lethal zone
  goals_state.push_back({&pose_c, true});

  goal_manager.setGoalStates(goals_state);
  EXPECT_EQ(goal_manager.getGoalsState().size(), 3);

  // Tolerance is high, one goal is invalid
  // all goals are valid
  goal_manager.removeInvalidGoals(tolerance, checker.get(), allow_unknow);
  EXPECT_EQ(goal_manager.getGoalsSet().size(), 3);
  EXPECT_EQ(goal_manager.getGoalsCoordinates().size(), 3);

  for (const auto & goal_state : goal_manager.getGoalsState()) {
    EXPECT_TRUE(goal_state.is_valid);
  }

  // Test with low tolerance, expect invalid goal to be filtered out
  goal_manager.setGoalStates(goals_state);
  int low_tolerance = 0.0f;
  goal_manager.removeInvalidGoals(low_tolerance, checker.get(), allow_unknow);

  EXPECT_EQ(goal_manager.getGoalsSet().size(), 2);
  EXPECT_EQ(goal_manager.getGoalsCoordinates().size(), 2);

  // expect last goal to be invalid
  for (const auto & goal_state : goal_manager.getGoalsState()) {
    if (goal_state.goal == goals_state[2].goal) {
      EXPECT_FALSE(goal_state.is_valid);
    } else {
      EXPECT_TRUE(goal_state.is_valid);
    }
  }

  // Prepare goals for expansion
  GoalStateVector expansion_goals;
  unsigned int test_goal_size = 16;

  for (unsigned int i = 0; i < test_goal_size; ++i) {
    auto goal = new NodeHybrid(i);
    goal->setPose(NodeHybrid::Coordinates(i, i, 0));
    expansion_goals.push_back({goal, true});
  }

  goal_manager.setGoalStates(expansion_goals);
  goal_manager.removeInvalidGoals(tolerance, checker.get(), allow_unknow);

  NodeVector coarse_check_goals;
  NodeVector fine_check_goals;

  // Resolution 1: everything coarse
  goal_manager.prepareGoalsForAnalyticExpansion(coarse_check_goals, fine_check_goals, 1);
  EXPECT_EQ(coarse_check_goals.size(), test_goal_size);
  EXPECT_TRUE(fine_check_goals.empty());

  // Resolution 4: every 4th coarse
  coarse_check_goals.clear();
  fine_check_goals.clear();

  goal_manager.prepareGoalsForAnalyticExpansion(coarse_check_goals, fine_check_goals, 4);
  EXPECT_EQ(coarse_check_goals.size(), 4u);  // indices 0, 4, 8, 12
  EXPECT_EQ(fine_check_goals.size(), 12u);

  delete costmapA;
  nav2_smac_planner::NodeHybrid::destroyStaticAssets();
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
