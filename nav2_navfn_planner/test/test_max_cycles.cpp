// Copyright (c) 2026, Jiarui Zou
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

#include <vector>

#include "gtest/gtest.h"
#include "nav2_navfn_planner/navfn.hpp"

TEST(NavFnMaxCyclesTest, PathExtractionRespectsCycleLimit)
{
  constexpr int map_size = 20;

  nav2_navfn_planner::NavFn navfn(map_size, map_size);
  std::vector<unsigned char> costmap(map_size * map_size, 0);

  int goal[2] = {2, 10};
  int start[2] = {10, 10};

  navfn.setCostmap(costmap.data());
  navfn.setGoal(goal);
  navfn.setStart(start);

  ASSERT_TRUE(
    navfn.calcNavFnDijkstra(
      []() {return false;}, true));

  // The potential is valid, but one extraction cycle cannot reach the goal.
  EXPECT_EQ(navfn.calcPath(1), 0);

  // A sufficient cycle budget extracts the complete path.
  EXPECT_GT(navfn.calcPath(map_size * 4), 0);
}
