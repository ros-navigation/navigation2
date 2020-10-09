// Copyright (c) 2020 Shivang Patel
// Copyright (c) 2020 Samsung Research
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

#include <string>
#include <vector>
#include <memory>

#include "gtest/gtest.h"
#include "smac_planner/collision_checker.hpp"

using namespace nav2_costmap_2d;  // NOLINT

TEST(collision_footprint, test_basic)
{
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0, 0, 0);

  geometry_msgs::msg::Point p1;
  p1.x = -0.5;
  p1.y = 0.0;
  geometry_msgs::msg::Point p2;
  p2.x = 0.0;
  p2.y = 0.5;
  geometry_msgs::msg::Point p3;
  p3.x = 0.5;
  p3.y = 0.0;
  geometry_msgs::msg::Point p4;
  p4.x = 0.0;
  p4.y = -0.5;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  smac_planner::GridCollisionChecker collision_checker(costmap_);
  collision_checker.setFootprint(footprint, false /*use footprint*/);
  collision_checker.inCollision(5.0, 5.0, 0.0, false);
  float cost = collision_checker.getCost();
  EXPECT_NEAR(cost, 0.0, 0.001);
  delete costmap_;
}

TEST(collision_footprint, test_point_cost)
{
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0, 0, 0);

  smac_planner::GridCollisionChecker collision_checker(costmap_);
  nav2_costmap_2d::Footprint footprint;
  collision_checker.setFootprint(footprint, true /*radius / pointcose*/);

  collision_checker.inCollision(5.0, 5.0, 0.0, false);
  float cost = collision_checker.getCost();
  EXPECT_NEAR(cost, 0.0, 0.001);
  delete costmap_;
}

TEST(collision_footprint, test_world_to_map)
{
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0, 0, 0);

  smac_planner::GridCollisionChecker collision_checker(costmap_);
  nav2_costmap_2d::Footprint footprint;
  collision_checker.setFootprint(footprint, true /*radius / point cost*/);

  unsigned int x, y;

  collision_checker.worldToMap(1.0, 1.0, x, y);

  collision_checker.inCollision(x, y, 0.0, false);
  float cost = collision_checker.getCost();

  EXPECT_NEAR(cost, 0.0, 0.001);

  costmap_->setCost(50, 50, 200);
  collision_checker.worldToMap(5.0, 5.0, x, y);

  collision_checker.inCollision(x, y, 0.0, false);
  EXPECT_NEAR(collision_checker.getCost(), 200.0, 0.001);
  delete costmap_;
}

TEST(collision_footprint, test_footprint_at_pose_with_movement)
{
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0, 0, 254);

  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmap_->setCost(i, j, 0);
    }
  }

  geometry_msgs::msg::Point p1;
  p1.x = -1.0;
  p1.y = 1.0;
  geometry_msgs::msg::Point p2;
  p2.x = 1.0;
  p2.y = 1.0;
  geometry_msgs::msg::Point p3;
  p3.x = 1.0;
  p3.y = -1.0;
  geometry_msgs::msg::Point p4;
  p4.x = -1.0;
  p4.y = -1.0;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  smac_planner::GridCollisionChecker collision_checker(costmap_);
  collision_checker.setFootprint(footprint, false /*use footprint*/);

  collision_checker.inCollision(50, 50, 0.0, false);
  float cost = collision_checker.getCost();
  EXPECT_NEAR(cost, 0.0, 0.001);

  collision_checker.inCollision(50, 49, 0.0, false);
  float up_value = collision_checker.getCost();
  EXPECT_NEAR(up_value, 254.0, 0.001);

  collision_checker.inCollision(50, 52, 0.0, false);
  float down_value = collision_checker.getCost();
  EXPECT_NEAR(down_value, 254.0, 0.001);
  delete costmap_;
}

TEST(collision_footprint, test_point_and_line_cost)
{
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(
    100, 100, 0.10000, 0, 0.0,
    0.0);

  costmap_->setCost(62, 50, 254);
  costmap_->setCost(39, 60, 254);

  geometry_msgs::msg::Point p1;
  p1.x = -1.0;
  p1.y = 1.0;
  geometry_msgs::msg::Point p2;
  p2.x = 1.0;
  p2.y = 1.0;
  geometry_msgs::msg::Point p3;
  p3.x = 1.0;
  p3.y = -1.0;
  geometry_msgs::msg::Point p4;
  p4.x = -1.0;
  p4.y = -1.0;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  smac_planner::GridCollisionChecker collision_checker(costmap_);
  collision_checker.setFootprint(footprint, false /*use footprint*/);

  collision_checker.inCollision(50, 50, 0.0, false);
  float value = collision_checker.getCost();
  EXPECT_NEAR(value, 0.0, 0.001);

  collision_checker.inCollision(49, 50, 0.0, false);
  float left_value = collision_checker.getCost();
  EXPECT_NEAR(left_value, 254.0, 0.001);

  collision_checker.inCollision(52, 50, 0.0, false);
  float right_value = collision_checker.getCost();
  EXPECT_NEAR(right_value, 254.0, 0.001);
  delete costmap_;
}
