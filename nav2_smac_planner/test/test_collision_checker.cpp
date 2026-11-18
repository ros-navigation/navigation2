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
#include "nav2_smac_planner/collision_checker.hpp"

using namespace nav2_costmap_2d;  // NOLINT

TEST(collision_footprint, test_basic)
{
  auto node = std::make_shared<nav2::LifecycleNode>("testA");
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

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
  collision_checker.setFootprint(footprint, false /*use footprint*/, 0.0);
  collision_checker.inCollision(5.0, 5.0, 0.0, false);
  float cost = collision_checker.getCost();
  EXPECT_NEAR(cost, 0.0, 0.001);
  delete costmap_;
}

TEST(collision_footprint, test_point_cost)
{
  auto node = std::make_shared<nav2::LifecycleNode>("testB");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0, 0, 0);

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
  nav2_costmap_2d::Footprint footprint;
  collision_checker.setFootprint(footprint, true /*radius / pointcose*/, 0.0);

  collision_checker.inCollision(5.0, 5.0, 0.0, false);
  float cost = collision_checker.getCost();
  EXPECT_NEAR(cost, 0.0, 0.001);
  delete costmap_;
}

TEST(collision_footprint, test_world_to_map)
{
  auto node = std::make_shared<nav2::LifecycleNode>("testC");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0, 0, 0);

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
  nav2_costmap_2d::Footprint footprint;
  collision_checker.setFootprint(footprint, true /*radius / point cost*/, 0.0);

  unsigned int x, y;

  collision_checker.worldToMap(1.0, 1.0, x, y);

  collision_checker.inCollision(x, y, 0.0, false);
  float cost = collision_checker.getCost();

  EXPECT_NEAR(cost, 0.0, 0.001);

  costmap->setCost(50, 50, 200);
  collision_checker.worldToMap(5.0, 5.0, x, y);

  collision_checker.inCollision(x, y, 0.0, false);
  EXPECT_NEAR(collision_checker.getCost(), 200.0, 0.001);
  delete costmap_;
}

TEST(collision_footprint, test_footprint_at_pose_with_movement)
{
  auto node = std::make_shared<nav2::LifecycleNode>("testD");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0, 0, 254);

  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmap_->setCost(i, j, 128);
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

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
  collision_checker.setFootprint(footprint, false /*use footprint*/, 0.0);

  EXPECT_FALSE(collision_checker.inCollision(50, 50, 0.0, false));
  float cost = collision_checker.getCost();
  EXPECT_NEAR(cost, 128.0, 0.001);

  EXPECT_TRUE(collision_checker.inCollision(50, 49, 0.0, false));
  float up_value = collision_checker.getCost();
  EXPECT_NEAR(up_value, 128.0, 0.001);  // center cost

  EXPECT_TRUE(collision_checker.inCollision(50, 52, 0.0, false));
  float down_value = collision_checker.getCost();
  EXPECT_NEAR(down_value, 128.0, 0.001);  // center cost

  EXPECT_TRUE(collision_checker.inCollision(11, 11, 0.0, false));
  float other_value = collision_checker.getCost();
  EXPECT_NEAR(other_value, 254.0, 0.001);  // center cost

  delete costmap_;
}

TEST(collision_footprint, test_point_and_line_cost)
{
  auto node = std::make_shared<nav2::LifecycleNode>("testE");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(
    100, 100, 0.10000, 0, 0.0, 128.0);

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

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
  collision_checker.setFootprint(footprint, false /*use footprint*/, 0.0);

  EXPECT_FALSE(collision_checker.inCollision(50, 50, 0.0, false));
  float value = collision_checker.getCost();
  EXPECT_NEAR(value, 128.0, 0.001);

  EXPECT_TRUE(collision_checker.inCollision(49, 50, 0.0, false));
  float left_value = collision_checker.getCost();
  EXPECT_NEAR(left_value, 128.0, 0.001);  // center cost

  EXPECT_TRUE(collision_checker.inCollision(52, 50, 0.0, false));
  float right_value = collision_checker.getCost();
  EXPECT_NEAR(right_value, 128.0, 0.001);  // center cost

  EXPECT_TRUE(collision_checker.inCollision(39, 60, 0.0, false));
  float other_value = collision_checker.getCost();
  EXPECT_NEAR(other_value, 254.0, 0.001);  // center cost

  delete costmap_;
}

// Regression test for #5996: poses are checked at their continuous coordinates
TEST(collision_footprint, test_fractional_pose_coordinates)
{
  auto node = std::make_shared<nav2::LifecycleNode>("testF");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(
    10, 10, 1.0, 0.0, 0.0, 0);

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);

  // A fractional pose belongs to the cell obtained by truncating its map coordinates.
  nav2_costmap_2d::Footprint empty_footprint;
  collision_checker.setFootprint(empty_footprint, true, 0.0);
  costmap->setCost(4, 5, 254);
  EXPECT_TRUE(collision_checker.inCollision(4.6, 5.1, 0.0, false));
  EXPECT_NEAR(collision_checker.getCost(), 254.0, 0.001);

  // A non-circular footprint must be translated to the exact continuous pose.
  costmap->setCost(4, 5, 0);
  costmap->setCost(3, 5, 254);
  geometry_msgs::msg::Point p1;
  p1.x = -0.4;
  p1.y = 0.4;
  geometry_msgs::msg::Point p2;
  p2.x = 0.4;
  p2.y = 0.4;
  geometry_msgs::msg::Point p3;
  p3.x = 0.4;
  p3.y = -0.4;
  geometry_msgs::msg::Point p4;
  p4.x = -0.4;
  p4.y = -0.4;
  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};
  collision_checker.setFootprint(footprint, false, 0.0);
  EXPECT_TRUE(collision_checker.inCollision(4.01, 4.99, 0.0, false));

  // The containing-cell cost must prevent an early exit before footprint checking.
  costmap->setCost(3, 5, 0);
  costmap->setCost(4, 5, 254);
  costmap->setCost(5, 5, 0);
  collision_checker.setFootprint(footprint, false, 128.0);
  EXPECT_TRUE(collision_checker.inCollision(4.6, 5.1, 0.0, false));

  delete costmap_;
}

TEST(collision_footprint, test_max_traversal_cost)
{
  auto node = std::make_shared<nav2::LifecycleNode>("testA");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  *costmap_ros->getCostmap() = nav2_costmap_2d::Costmap2D(100, 100, 0.05, 0, 0, 0);
  costmap_ros->getCostmap()->setCost(50, 50, 245);
  costmap_ros->getCostmap()->setCost(70, 70, NO_INFORMATION);

  nav2_smac_planner::GridCollisionChecker checker(costmap_ros, 72, node);
  checker.setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  // Disabled by default, and when set above the max non-obstacle cost
  EXPECT_FALSE(checker.inCollision(50.0f, 50.0f, 0.0f, false));
  checker.setMaxCost(300.0f, 20.0f, 5.0f, 5.0f, 95.0f, 95.0f);
  EXPECT_FALSE(checker.inCollision(50.0f, 50.0f, 0.0f, false));

  // Above the max cost is blocked, except within the relief radius of the start or goal
  checker.setMaxCost(240.0f, 20.0f, 5.0f, 5.0f, 95.0f, 95.0f);
  EXPECT_TRUE(checker.inCollision(50.0f, 50.0f, 0.0f, false));
  EXPECT_TRUE(checker.inCollision(50u * 100u + 50u, false));
  EXPECT_FALSE(checker.exceedsMaxCost(240.0f, 50.0f, 50.0f));  // at max cost
  EXPECT_FALSE(checker.exceedsMaxCost(252.0f, 24.0f, 5.0f));  // in start relief
  EXPECT_FALSE(checker.exceedsMaxCost(252.0f, 95.0f, 76.0f));  // in goal relief
  EXPECT_TRUE(checker.exceedsMaxCost(252.0f, 26.0f, 5.0f));  // outside relief
  EXPECT_TRUE(checker.exceedsMaxCost(253.0f, 5.0f, 5.0f));  // lethal, even in relief

  // Unknown follows traverse unknown in both overloads
  EXPECT_FALSE(checker.inCollision(70.0f, 70.0f, 0.0f, true));
  EXPECT_TRUE(checker.inCollision(70.0f, 70.0f, 0.0f, false));
  EXPECT_FALSE(checker.inCollision(70u * 100u + 70u, true));
  EXPECT_TRUE(checker.inCollision(70u * 100u + 70u, false));

  // With a footprint, only the center cost is limited, not the footprint's
  geometry_msgs::msg::Point a, b, c, d;
  a.x = -0.15; a.y = -0.15; b.x = 0.15; b.y = -0.15;
  c.x = 0.15; c.y = 0.15; d.x = -0.15; d.y = 0.15;
  checker.setFootprint({a, b, c, d}, false, 0.0);
  EXPECT_FALSE(checker.inCollision(47.0f, 50.0f, 0.0f, false));
  EXPECT_TRUE(checker.inCollision(50.0f, 50.0f, 0.0f, false));
}

TEST(collision_footprint, test_pose_cell_and_bounds)
{
  auto node = std::make_shared<nav2::LifecycleNode>("testA");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  *costmap_ros->getCostmap() = nav2_costmap_2d::Costmap2D(100, 100, 0.05, 0, 0, 0);
  costmap_ros->getCostmap()->setCost(16, 50, 254);

  nav2_smac_planner::GridCollisionChecker checker(costmap_ros, 72, node);
  checker.setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  // Cell i spans [i, i+1) in continuous map coordinates
  EXPECT_FALSE(checker.inCollision(15.9f, 50.0f, 0.0f, false));
  EXPECT_TRUE(checker.inCollision(16.0f, 50.0f, 0.0f, false));
  EXPECT_TRUE(checker.inCollision(16.9f, 50.0f, 0.0f, false));

  // Poses must be within the map, excluding its border
  EXPECT_FALSE(checker.inCollision(3.0f, 50.0f, 0.0f, false));
  EXPECT_TRUE(checker.inCollision(2.9f, 50.0f, 0.0f, false));
  EXPECT_FALSE(checker.inCollision(96.9f, 50.0f, 0.0f, false));
  EXPECT_TRUE(checker.inCollision(97.0f, 50.0f, 0.0f, false));
  EXPECT_TRUE(checker.inCollision(50.0f, 97.0f, 0.0f, false));
  EXPECT_TRUE(checker.inCollision(50u * 100u + 2u, false));
  EXPECT_FALSE(checker.inCollision(50u * 100u + 3u, false));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
