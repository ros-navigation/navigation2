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
#include "nav2_route/collision_checker.hpp"
#include "nav2_route/breadth_first_search.hpp"

using namespace nav2_costmap_2d; //NOLINT
using namespace nav2_route; //NOLINT

TEST(test_breadth_first_search, bfs_test)
{
  unsigned int x_size = 10;
  unsigned int y_size = 10;

  Costmap2D costmap(x_size, y_size, 0.0, 0.0, 1);
  CollisionChecker collision_checker(&costmap);

  BreadthFirstSearch bfs;

  bfs.setCollisionChecker(&collision_checker);

  bfs.setStart(0u, 0u);
  std::vector<unsigned int> mxs = {2u, 5u};
  std::vector<unsigned int> mys = {3u, 5u};

  bfs.setGoals(mxs, mys);

  BreadthFirstSearch::CoordinateVector path;
  bool result = bfs.search(path);

  EXPECT_TRUE(result);

  for (const auto pose : path) {
    std::cout << pose.x << " " << pose.y << std::endl;
  }
}
