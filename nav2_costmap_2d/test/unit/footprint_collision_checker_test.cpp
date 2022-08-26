// Copyright (c) 2020 Shivang Patel
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
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/footprint.hpp"

TEST(collision_footprint, test_basic)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

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

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  auto value = collision_checker.footprintCostAtPose(5.0, 5.0, 0.0, footprint);

  EXPECT_NEAR(value, 0.0, 0.001);
}

TEST(collision_footprint, test_point_cost)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  auto value = collision_checker.pointCost(50, 50);

  EXPECT_NEAR(value, 0.0, 0.001);
}

TEST(collision_footprint, test_world_to_map)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  unsigned int x, y;

  collision_checker.worldToMap(1.0, 1.0, x, y);

  auto value = collision_checker.pointCost(x, y);

  EXPECT_NEAR(value, 0.0, 0.001);

  costmap_->setCost(50, 50, 200);
  collision_checker.worldToMap(5.0, 5.0, x, y);

  EXPECT_NEAR(collision_checker.pointCost(x, y), 200.0, 0.001);
}

TEST(collision_footprint, test_footprint_at_pose_with_movement)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 254);

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

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  auto value = collision_checker.footprintCostAtPose(5.0, 5.0, 0.0, footprint);
  EXPECT_NEAR(value, 0.0, 0.001);

  auto up_value = collision_checker.footprintCostAtPose(5.0, 4.9, 0.0, footprint);
  EXPECT_NEAR(up_value, 254.0, 0.001);

  auto down_value = collision_checker.footprintCostAtPose(5.0, 5.2, 0.0, footprint);
  EXPECT_NEAR(down_value, 254.0, 0.001);
}

TEST(collision_footprint, test_point_and_line_cost)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.10000, 0, 0.0, 0.0);

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

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  auto value = collision_checker.footprintCostAtPose(5.0, 5.0, 0.0, footprint);
  EXPECT_NEAR(value, 0.0, 0.001);

  auto left_value = collision_checker.footprintCostAtPose(4.9, 5.0, 0.0, footprint);
  EXPECT_NEAR(left_value, 254.0, 0.001);

  auto right_value = collision_checker.footprintCostAtPose(5.2, 5.0, 0.0, footprint);
  EXPECT_NEAR(right_value, 254.0, 0.001);
}

TEST(collision_footprint, not_enough_points)
{
  geometry_msgs::msg::Point p1;
  p1.x = 2.0;
  p1.y = 2.0;

  geometry_msgs::msg::Point p2;
  p2.x = -2.0;
  p2.y = -2.0;

  std::vector<geometry_msgs::msg::Point> footprint = {p1, p2};
  double min_dist = 0.0;
  double max_dist = 0.0;

  nav2_costmap_2d::calculateMinAndMaxDistances(footprint, min_dist, max_dist);
  EXPECT_EQ(min_dist, std::numeric_limits<double>::max());
  EXPECT_EQ(max_dist, 0.0f);
}

TEST(collision_footprint, to_point_32) {
  geometry_msgs::msg::Point p;
  p.x = 123.0;
  p.y = 456.0;
  p.z = 789.0;

  geometry_msgs::msg::Point32 p32;
  p32 = nav2_costmap_2d::toPoint32(p);
  EXPECT_NEAR(p.x, p32.x, 1e-5);
  EXPECT_NEAR(p.y, p32.y, 1e-5);
  EXPECT_NEAR(p.z, p32.z, 1e-5);
}

TEST(collision_footprint, to_polygon) {
  geometry_msgs::msg::Point p1;
  p1.x = 1.2;
  p1.y = 3.4;
  p1.z = 5.1;

  geometry_msgs::msg::Point p2;
  p2.x = -5.6;
  p2.y = -7.8;
  p2.z = -9.1;
  std::vector<geometry_msgs::msg::Point> pts = {p1, p2};

  geometry_msgs::msg::Polygon poly;
  poly = nav2_costmap_2d::toPolygon(pts);

  EXPECT_EQ(2u, sizeof(poly.points) / sizeof(poly.points[0]));
  EXPECT_NEAR(poly.points[0].x, p1.x, 1e-5);
  EXPECT_NEAR(poly.points[0].y, p1.y, 1e-5);
  EXPECT_NEAR(poly.points[0].z, p1.z, 1e-5);
  EXPECT_NEAR(poly.points[1].x, p2.x, 1e-5);
  EXPECT_NEAR(poly.points[1].y, p2.y, 1e-5);
  EXPECT_NEAR(poly.points[1].z, p2.z, 1e-5);
}

TEST(collision_footprint, make_footprint_from_string) {
  std::vector<geometry_msgs::msg::Point> footprint;
  bool result = nav2_costmap_2d::makeFootprintFromString(
    "[[1, 2.2], [.3, -4e4], [-.3, -4e4], [-1, 2.2]]", footprint);
  EXPECT_EQ(result, true);
  EXPECT_EQ(4u, footprint.size());
  EXPECT_NEAR(footprint[0].x, 1.0, 1e-5);
  EXPECT_NEAR(footprint[0].y, 2.2, 1e-5);
  EXPECT_NEAR(footprint[1].x, 0.3, 1e-5);
  EXPECT_NEAR(footprint[1].y, -4e4, 1e-5);
  EXPECT_NEAR(footprint[2].x, -0.3, 1e-5);
  EXPECT_NEAR(footprint[2].y, -4e4, 1e-5);
  EXPECT_NEAR(footprint[3].x, -1.0, 1e-5);
  EXPECT_NEAR(footprint[3].y, 2.2, 1e-5);
}

TEST(collision_footprint, make_footprint_from_string_parse_error) {
  std::vector<geometry_msgs::msg::Point> footprint;
  bool result = nav2_costmap_2d::makeFootprintFromString(
    "[[bad_string", footprint);
  EXPECT_EQ(result, false);
}

TEST(collision_footprint, make_footprint_from_string_two_points_error) {
  std::vector<geometry_msgs::msg::Point> footprint;
  bool result = nav2_costmap_2d::makeFootprintFromString(
    "[[1, 2.2], [.3, -4e4]", footprint);
  EXPECT_EQ(result, false);
}

TEST(collision_footprint, make_footprint_from_string_not_pairs) {
  std::vector<geometry_msgs::msg::Point> footprint;
  bool result = nav2_costmap_2d::makeFootprintFromString(
    "[[1, 2.2], [.3, -4e4], [-.3, -4e4], [-1, 2.2, 5.6]]", footprint);
  EXPECT_EQ(result, false);
}
