// Copyright (c) 2023 Open Navigation LLC
// Copyright 2024 Stevedan Ogochukwu Omodolor Omodia
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


#include "gtest/gtest.h"
#include "nav2_smac_planner/goal_manager.hpp"

using namespace nav2_smac_planner; // NOLINT

using GoalManagerHybrid = GoalManager<NodeHybrid>;
using NodePtr = NodeHybrid *;
using NodeVector = GoalManagerHybrid::NodeVector;
using CoordinateVector = GoalManagerHybrid::CoordinateVector;

TEST(GoalManagerTest, GoalManagerBehavior)
{
  auto goal_manager = GoalManagerHybrid();
  EXPECT_TRUE(goal_manager.goalsIsEmpty());

  // Setup test goals
  NodeHybrid poseA(48);
  NodeHybrid poseB(49);
  NodeHybrid::Coordinates coordA(0, 0, 0);
  NodeHybrid::Coordinates coordB(0, 0, 10);

  NodeVector nodes{&poseA, &poseB};
  CoordinateVector coords{coordA, coordB};

  goal_manager.populate(nodes, coords);
  EXPECT_FALSE(goal_manager.goalsIsEmpty());
  EXPECT_EQ(goal_manager.getGoalsState().size(), 2);
  EXPECT_EQ(goal_manager.getGoalsSet().size(), 0);
  EXPECT_EQ(goal_manager.getGoalsCoordinates().size(), 0);

  // Filter: All valid
  bool all_invalid = true;
  auto validCheck = [](const NodePtr &) {return true;};
  goal_manager.filterAndStoreTryableGoals(validCheck, all_invalid);

  EXPECT_FALSE(all_invalid);
  EXPECT_EQ(goal_manager.getGoalsSet().size(), 2);
  EXPECT_EQ(goal_manager.getGoalsCoordinates().size(), 2);
  for (const auto & goal_state : goal_manager.getGoalsState()) {
    EXPECT_TRUE(goal_state.is_valid);
  }

  // Re-populate and validate reset state
  goal_manager.populate(nodes, coords);
  EXPECT_EQ(goal_manager.getGoalsSet().size(), 0);
  EXPECT_EQ(goal_manager.getGoalsCoordinates().size(), 0);

  // Filter: All invalid
  auto invalidCheck = [](const NodePtr &) {return false;};
  goal_manager.filterAndStoreTryableGoals(invalidCheck, all_invalid);

  EXPECT_TRUE(all_invalid);
  EXPECT_EQ(goal_manager.getGoalsSet().size(), 0);
  EXPECT_EQ(goal_manager.getGoalsCoordinates().size(), 0);
  for (const auto & goal_state : goal_manager.getGoalsState()) {
    EXPECT_FALSE(goal_state.is_valid);
  }

  // Prepare goals for expansion
  std::vector<std::unique_ptr<NodeHybrid>> node_storage;
  NodeVector test_nodes;
  CoordinateVector test_coords;
  size_t test_goal_size = 16;

  for (unsigned int i = 0; i < test_goal_size; ++i) {
    NodeHybrid * node = new NodeHybrid(i);
    test_nodes.push_back(node);
    test_coords.emplace_back(i, i, i);
  }

  goal_manager.populate(test_nodes, test_coords);

  NodeVector coarse, fine;
  goal_manager.filterAndStoreTryableGoals(validCheck, all_invalid);  // ensure valid

  // Test with resolution = 1 (everything coarse)
  goal_manager.prepareGoalsForExpansion(coarse, fine, 1);
  EXPECT_EQ(coarse.size(), test_goal_size);
  EXPECT_EQ(fine.size(), 0);

  // Test with resolution = 4 (every 4th is coarse)
  coarse.clear();
  fine.clear();
  goal_manager.prepareGoalsForExpansion(coarse, fine, 4);
  EXPECT_EQ(coarse.size(), 4);  // indices 0, 4, 8, 12
  EXPECT_EQ(fine.size(), 12);  // the rest
}


TEST(GoalManagerTest, PartialValidGoals)
{
 auto goal_manager = GoalManagerHybrid();

 NodeVector nodes;
 CoordinateVector coords;
 size_t goal_count = 16;

 for (unsigned int i = 0; i < goal_count; ++i) {
    NodeHybrid * node = new NodeHybrid(i);
    nodes.push_back(node);
    coords.emplace_back(i, i, i);
 }

  // Populate with 6 goals
  goal_manager.populate(nodes, coords);

  // Mark even-indexed goals as valid, odd as invalid
  auto validFn = [](const NodePtr & node) {
      return node->getIndex() % 2 == 0;
    };

  bool all_invalid = true;
  goal_manager.filterAndStoreTryableGoals(validFn, all_invalid);

  EXPECT_FALSE(all_invalid);
  EXPECT_EQ(goal_manager.getGoalsState().size(), goal_count);

  // Check flags and collect expected valid ones
  std::set<NodePtr> expected_valid_set;
  std::vector<NodeHybrid::Coordinates> expected_valid_coords;

  for (unsigned int i = 0; i < goal_count; ++i) {
    bool expected_valid = (i % 2 == 0);
    const auto & goal_state = goal_manager.getGoalsState()[i];
    EXPECT_EQ(goal_state.is_valid, expected_valid);

    if (expected_valid) {
      expected_valid_set.insert(goal_state.goal);
      expected_valid_coords.push_back(goal_state.goal->pose);
    }
  }
}
