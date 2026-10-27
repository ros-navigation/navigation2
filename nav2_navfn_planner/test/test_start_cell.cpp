// Copyright (c) 2026, Nisarg
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

#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(NavfnTest, testPlanFromStartNextToInscribedCells)
{
  constexpr unsigned int size = 60;
  constexpr double resolution = 0.05;
  constexpr unsigned int block_min = 10;
  constexpr unsigned int block_max = 20;

  auto node = std::make_shared<nav2::LifecycleNode>("NavfnStartCellTest");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  costmap->resizeMap(size, size, resolution, 0.0, 0.0);
  for (unsigned int x = 0; x < size; ++x) {
    for (unsigned int y = 0; y < size; ++y) {
      const bool in_block = x >= block_min && x <= block_max && y >= block_min && y <= block_max &&
        !(x == block_min && y == block_min);
      costmap->setCost(
        x, y, in_block ?
        nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE : nav2_costmap_2d::FREE_SPACE);
    }
  }

  auto planner = std::make_unique<nav2_navfn_planner::NavfnPlanner>();
  auto tf = nav2::create_transform_buffer(node);
  planner->configure(node, "test", tf, costmap_ros);
  planner->activate();

  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "map";
  start.pose.position.x = 10.8 * resolution;
  start.pose.position.y = 10.8 * resolution;
  start.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped goal = start;
  goal.pose.position.x = 50.0 * resolution;
  goal.pose.position.y = 5.0 * resolution;

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my));
  ASSERT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE);

  nav_msgs::msg::Path path;
  EXPECT_NO_THROW(path = planner->createPlan(start, goal, {}, [] {return false;}));
  EXPECT_FALSE(path.poses.empty());

  planner->deactivate();
  planner->cleanup();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
