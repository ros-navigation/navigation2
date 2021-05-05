// Copyright (c) 2021 Samsung Research Russia
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

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(CopyWindow, copyValidWindow)
{
  nav2_costmap_2d::Costmap2D src(10, 10, 0.1, 0.0, 0.0);
  nav2_costmap_2d::Costmap2D dst(5, 5, 0.2, 100.0, 100.0);
  // Adding 2 marked cells to source costmap
  src.setCost(2, 2, 100);
  src.setCost(5, 5, 200);

  ASSERT_TRUE(dst.copyWindow(src, 2, 2, 6, 6, 0, 0));
  // Check that both marked cells were copied to destination costmap
  ASSERT_TRUE(dst.getCost(0, 0) == 100);
  ASSERT_TRUE(dst.getCost(3, 3) == 200);
}

TEST(CopyWindow, copyInvalidWindow)
{
  nav2_costmap_2d::Costmap2D src(10, 10, 0.1, 0.0, 0.0);
  nav2_costmap_2d::Costmap2D dst(5, 5, 0.2, 100.0, 100.0);

  // Case1: incorrect source bounds
  ASSERT_FALSE(dst.copyWindow(src, 9, 9, 11, 11, 0, 0));
  // Case2: incorrect destination bounds
  ASSERT_FALSE(dst.copyWindow(src, 0, 0, 1, 1, 5, 5));
  ASSERT_FALSE(dst.copyWindow(src, 0, 0, 6, 6, 0, 0));
}
