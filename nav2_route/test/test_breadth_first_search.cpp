// Copyright (c) 2023 Joshua Wallace
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
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_route/breadth_first_search.hpp"

using namespace nav2_costmap_2d; //NOLINT
using namespace nav2_route; //NOLINT

class DSTestFixture : public ::testing::Test
{
public:
  void initialize(unsigned int x_size, unsigned int y_size)
  {
    costmap = std::make_unique<Costmap2D>(x_size, y_size, 0.0, 0.0, 1);

    ds.initialize(costmap, 200);
  }

  DijkstraSearch ds;
  std::shared_ptr<Costmap2D> costmap;
};

TEST_F(DSTestFixture, free_space)
{
  initialize(100u, 100u);

  ds.setStart(0u, 0u);

  std::vector<nav2_costmap_2d::MapLocation> goals;
  goals.push_back({50u, 50u});
  ds.setGoals(goals);

  unsigned int goal = 0;
  ASSERT_NO_THROW(ds.search(goal));

  EXPECT_EQ(costmap->getIndex(goals[goal].x, goals[goal].y), costmap->getIndex(50u, 50u));
  ds.clearGraph();
}

TEST_F(DSTestFixture, wall)
{
  initialize(10u, 10u);

  unsigned int mx_obs = 3;
  for (unsigned int my_obs = 0; my_obs < costmap->getSizeInCellsY(); ++my_obs) {
    costmap->setCost(mx_obs, my_obs, LETHAL_OBSTACLE);
  }

  ds.setStart(0u, 0u);

  std::vector<nav2_costmap_2d::MapLocation> goals;
  goals.push_back({5u, 0u});
  goals.push_back({0u, 4u});
  ds.setGoals(goals);

  unsigned int goal = 0;
  EXPECT_NO_THROW(ds.search(goal));

  EXPECT_EQ(costmap->getIndex(goals[goal].x, goals[goal].y), costmap->getIndex(0u, 4u));
  ds.clearGraph();
}

TEST_F(DSTestFixture, ray_trace)
{
  initialize(10u, 10u);

  ds.setStart(0u, 0u);

  std::vector<nav2_costmap_2d::MapLocation> goals;
  goals.push_back({5u, 5u});
  ds.setGoals(goals);

  bool result = ds.isFirstGoalVisible();
  EXPECT_TRUE(result);
  ds.clearGraph();

  ds.setStart(0u, 0u);
  ds.setGoals(goals);
  costmap->setCost(2u, 2u, LETHAL_OBSTACLE);
  result = ds.isFirstGoalVisible();
  EXPECT_FALSE(result);

  ds.clearGraph();
}
