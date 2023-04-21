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
#include "nav2_route/collision_checker.hpp"
#include "nav2_route/breadth_first_search.hpp"

using namespace nav2_costmap_2d; //NOLINT
using namespace nav2_route; //NOLINT

class BFSTestFixture : public ::testing::Test
{
public:
  void initialize(unsigned int x_size, unsigned int y_size)
  {
    costmap = std::make_unique<Costmap2D>(x_size, y_size, 0.0, 0.0, 1);
    collision_checker = std::make_unique<CollisionChecker>(costmap.get());

    bfs.initialize(1000);
    bfs.setCollisionChecker(collision_checker.get());
  }

  BreadthFirstSearch bfs;
  std::unique_ptr<Costmap2D> costmap;
  std::unique_ptr<CollisionChecker> collision_checker;
  BreadthFirstSearch::CoordinateVector path;
};

TEST_F(BFSTestFixture, free_space)
{
  initialize(10u, 10u);

  bfs.setStart(0u, 0u);
  std::vector<unsigned int> mxs = {2u, 5u};
  std::vector<unsigned int> mys = {3u, 5u};

  bfs.setGoals(mxs, mys);
  bool result = bfs.search(path);

  EXPECT_TRUE(result);
  EXPECT_EQ(path.begin()->x, 2);
  EXPECT_EQ(path.begin()->y, 3);
  path.clear();
}

TEST_F(BFSTestFixture, wall)
{
  initialize(10u, 10u);

  unsigned int mx = 3;
  for(unsigned int my=0; my < costmap->getSizeInCellsY() -1; ++my) {
    costmap->setCost(mx, my, LETHAL_OBSTACLE);
  }

  bfs.setStart(0u, 0u);

  std::vector<unsigned int> mxs = {2u, 5u};
  std::vector<unsigned int> mys = {8u, 0u};

  bfs.setGoals(mxs, mys);

  bool result = bfs.search(path);
  EXPECT_TRUE(result);

  EXPECT_EQ(path.begin()->x, 2);
  EXPECT_EQ(path.begin()->y, 8);
  path.clear();
}