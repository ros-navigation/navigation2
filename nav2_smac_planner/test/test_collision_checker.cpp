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
#include "nav2_util/lifecycle_node.hpp"

using namespace nav2_costmap_2d;  // NOLINT

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(collision_footprint, test_basic)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testA");
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
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testB");
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
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testC");
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
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testD");
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
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testE");
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
