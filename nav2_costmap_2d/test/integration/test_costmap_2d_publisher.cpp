//
// Created by josh on 11/7/22.
//

// Copyright (c) 2022 Joshua Wallace
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

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
//#include "../testing_helper.hpp"

//class DummyCostmap2DLayer : public nav2_costmap_2d::Costmap2D
//{
//public:
//  DummyCostmap2DLayer() {}
//};

TEST(costmap_publisher_test, dummy_test)
{
  EXPECT_EQ(1, 1);
}

//int main(int argc, char ** argv)
//{
//  // Create a a fake costmap
//  nav2_costmap_2d::Costmap2D costmap_2d(100, 100, 0.05, 0, 0, nav2_costmap_2d::FREE_SPACE);
//  costmap_2d.setCost(5, 5, nav2_costmap_2d::LETHAL_OBSTACLE);
//
////  printMap(costmap_2d);
//
//
//  testing::InitGoogleTest(&argc, argv);
//  return RUN_ALL_TESTS();
//}