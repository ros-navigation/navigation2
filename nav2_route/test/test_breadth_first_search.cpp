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

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_route/breadth_first_search.hpp"

using namespace nav2_costmap_2d; //NOLINT
using namespace nav2_route; //NOLINT

class BFSTestFixture : public ::testing::Test
{
public:
  void initialize(unsigned int x_size, unsigned int y_size)
  {
    costmap = std::make_unique<Costmap2D>(x_size, y_size, 0.0, 0.0, 1);

    bfs.setCostmap(costmap.get());
  }

  BreadthFirstSearch bfs;
  std::unique_ptr<Costmap2D> costmap;
};

TEST_F(BFSTestFixture, free_space)
{
  initialize(10u, 10u);

  bfs.setStart(0u, 0u);

  unsigned int mx = 1u;
  unsigned int my = 1u;
  bfs.setGoal(mx, my);

  bool result = bfs.search();
  EXPECT_TRUE(result);
  bfs.clearGraph();
}

TEST_F(BFSTestFixture, wall)
{
  initialize(10u, 10u);

  unsigned int mx_obs = 3;
  for(unsigned int my_obs=0; my_obs < costmap->getSizeInCellsY(); ++my_obs) {
    costmap->setCost(mx_obs, my_obs, LETHAL_OBSTACLE);
  }

  bfs.setStart(0u, 0u);

  unsigned int mx = 5u;
  unsigned int my = 0u;

  bfs.setGoal(mx, my);

  bool result = bfs.search();
  EXPECT_FALSE(result);
  bfs.clearGraph();
}

TEST_F(BFSTestFixture, ray_trace)
{
  initialize(10u, 10u);

  bfs.setStart(0u, 0u);
  bfs.setGoal(5u, 5u);

  bool result = bfs.isNodeVisible();
  EXPECT_TRUE(result);

  costmap->setCost(2u, 2u, LETHAL_OBSTACLE);
  result = bfs.isNodeVisible();
  EXPECT_FALSE(result);

  bfs.clearGraph();
}
