#include <string>
#include <vector>
#include <memory>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/collision_footprint.hpp"

TEST(collision_footprint, footprintCostWithPose)
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

  nav2_costmap_2d::CollisionFootprint collision_footprint(costmap_);

  auto value = collision_footprint.footprintCostWithPose(5.0, 5.0, 0.0, footprint);

  EXPECT_NEAR(value, 0.0, 0.001);
}

TEST(collision_footprint, pointCost)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  nav2_costmap_2d::CollisionFootprint collision_footprint(costmap_);

  auto value = collision_footprint.pointCost(50, 50);

  EXPECT_NEAR(value, 0.0, 0.001);
}

TEST(collision_footprint, worldToMap)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  nav2_costmap_2d::CollisionFootprint collision_footprint(costmap_);

  unsigned int x, y;

  collision_footprint.worldToMap(5.0, 5.0, x, y);

  auto value = collision_footprint.pointCost(x, y);

  EXPECT_NEAR(value, 0.0, 0.001);
}

TEST(collision_footprint, footprintCostWithPoseObstacle)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  for (unsigned int i = 45; i <= 65; ++i) {
    for (unsigned int j = 45; j <= 65; ++j) {
      costmap_->setCost(i, j, 254);
    }
  }

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

  nav2_costmap_2d::CollisionFootprint collision_footprint(costmap_);

  auto value = collision_footprint.footprintCostWithPose(5.0, 5.0, 0.0, footprint);

  EXPECT_NEAR(value, 254.0, 0.001);
}
