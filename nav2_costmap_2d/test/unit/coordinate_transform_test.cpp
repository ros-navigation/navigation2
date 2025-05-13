// Copyright (c) 2025 Open Navigation LLC
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
// limitations under the License. Reserved.

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"


/**
 * Test for mapToWorldNoBounds
 */

TEST(mapToWorldNoBounds, MapToWorldNoBoundsNegativeMapCoords)
{
  double wx, wy;

  std::unique_ptr<nav2_costmap_2d::Costmap2D> map;

  map = std::make_unique<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 0.0, 0.0);
  map->mapToWorldNoBounds(-1, -1, wx, wy);
  EXPECT_DOUBLE_EQ(wx, -0.5);
  EXPECT_DOUBLE_EQ(wy, -0.5);

  map = std::make_unique<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 1.0, 2.0);
  map->mapToWorldNoBounds(-5, -5, wx, wy);
  EXPECT_DOUBLE_EQ(wx, -3.5);
  EXPECT_DOUBLE_EQ(wy, -2.5);

  map = std::make_unique<nav2_costmap_2d::Costmap2D>(10, 10, 2.0, 3.0, 4.0);
  map->mapToWorldNoBounds(-10, -10, wx, wy);
  EXPECT_DOUBLE_EQ(wx, -16.0);
  EXPECT_DOUBLE_EQ(wy, -15.0);
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
