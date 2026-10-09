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

TEST(collision_footprint, test_footprint_at_exact_pose)
{
  auto node = std::make_shared<nav2::LifecycleNode>("testF");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(
    100, 100, 0.05, 0.0, 0.0, 0);

  // Obstacles just beside and ahead of a 2m footprint's tip at (50.5, 50.5, 0)
  costmap_->setCost(90, 52, 254);
  costmap_->setCost(91, 50, 254);

  geometry_msgs::msg::Point p1;
  p1.x = -0.05;
  p1.y = 0.05;
  geometry_msgs::msg::Point p2;
  p2.x = 2.02;
  p2.y = 0.05;
  geometry_msgs::msg::Point p3;
  p3.x = 2.02;
  p3.y = -0.05;
  geometry_msgs::msg::Point p4;
  p4.x = -0.05;
  p4.y = -0.05;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
  collision_checker.setFootprint(footprint, false /*use footprint*/, 0.0);

  // Cell center and bin 0 heading is free
  EXPECT_FALSE(collision_checker.inCollision(50, 50, 0.0, false));
  EXPECT_FALSE(collision_checker.inCollisionAtPose(50.5, 50.5, 0.0, false));

  // 2 degrees snaps to bin 0, but is in collision
  const double yaw = 2.0 * M_PI / 180.0;
  EXPECT_TRUE(collision_checker.inCollisionAtPose(50.5, 50.5, yaw, false));
  EXPECT_NEAR(collision_checker.getCost(), 0.0, 0.001);  // center cost

  // Ahead of the cell center is in collision, but not when snapped to it
  EXPECT_FALSE(collision_checker.inCollision(50.9, 50.5, 0.0, false));
  EXPECT_TRUE(collision_checker.inCollisionAtPose(50.9, 50.5, 0.0, false));

  // Exact heading matches a precomputed bin
  EXPECT_EQ(
    collision_checker.inCollision(50, 50, 18.0, false),
    collision_checker.inCollisionAtPose(50.5, 50.5, M_PI / 2.0, false));

  // Out of map and center cost checks
  EXPECT_TRUE(collision_checker.inCollisionAtPose(150.0, 50.0, yaw, false));
  EXPECT_TRUE(collision_checker.inCollisionAtPose(100.0, 50.0, yaw, false));
  EXPECT_TRUE(collision_checker.inCollisionAtPose(-0.5, 50.0, yaw, false));
  EXPECT_TRUE(collision_checker.inCollisionAtPose(90.5, 52.5, 0.0, false));
  EXPECT_NEAR(collision_checker.getCost(), 254.0, 0.001);  // center cost

  delete costmap_;
}

TEST(collision_footprint, test_pose_check_matches_grid_check)
{
  auto node = std::make_shared<nav2::LifecycleNode>("testG");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(
    40, 40, 0.1, 0.0, 0.0, 0);

  // Mix of free, inflated, inscribed, lethal and unknown cells
  for (unsigned int i = 0; i < 40; ++i) {
    for (unsigned int j = 0; j < 40; ++j) {
      const unsigned int k = (i * 7 + j * 13) % 41;
      if (k == 0) {
        costmap_->setCost(i, j, 254);
      } else if (k == 1) {
        costmap_->setCost(i, j, 253);
      } else if (k <= 4) {
        costmap_->setCost(i, j, 255);
      } else if (k <= 12) {
        costmap_->setCost(i, j, 50 + 20 * (k - 5));
      }
    }
  }

  geometry_msgs::msg::Point p1;
  p1.x = -0.17;
  p1.y = 0.13;
  geometry_msgs::msg::Point p2;
  p2.x = 0.33;
  p2.y = 0.13;
  geometry_msgs::msg::Point p3;
  p3.x = 0.33;
  p3.y = -0.13;
  geometry_msgs::msg::Point p4;
  p4.x = -0.17;
  p4.y = -0.13;
  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmap_;

  // A grid node at cell (i, j) and bin b is checked at the cell center, so
  // the exact pose check at (i + 0.5, j + 0.5) and yaw of bin b must agree
  const double bin_size = 2.0 * M_PI / 72.0;
  for (const bool radius : {false, true}) {
    for (const double possible_collision_cost : {0.0, 100.0}) {
      nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
      collision_checker.setFootprint(footprint, radius, possible_collision_cost);
      for (const bool traverse_unknown : {false, true}) {
        for (unsigned int i = 0; i < 40; ++i) {
          for (unsigned int j = 0; j < 40; ++j) {
            for (unsigned int b = 0; b < 72; b += 7) {
              const bool grid = collision_checker.inCollision(i, j, b, traverse_unknown);
              const float grid_cost = collision_checker.getCost();
              const bool pose = collision_checker.inCollisionAtPose(
                i + 0.5f, j + 0.5f, b * bin_size, traverse_unknown);
              ASSERT_EQ(grid, pose) << "cell (" << i << ", " << j << ") bin " << b <<
                " radius " << radius << " possible cost " << possible_collision_cost <<
                " traverse unknown " << traverse_unknown;
              ASSERT_EQ(grid_cost, collision_checker.getCost());
            }
          }
        }
      }
    }
  }

  // Poses outside of the map are in collision
  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
  collision_checker.setFootprint(footprint, false, 0.0);
  EXPECT_TRUE(collision_checker.inCollisionAtPose(-0.01f, 20.0f, 0.0, true));
  EXPECT_TRUE(collision_checker.inCollisionAtPose(20.0f, -0.01f, 0.0, true));
  EXPECT_TRUE(collision_checker.inCollisionAtPose(40.0f, 20.0f, 0.0, true));
  EXPECT_TRUE(collision_checker.inCollisionAtPose(20.0f, 40.0f, 0.0, true));

  delete costmap_;
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
