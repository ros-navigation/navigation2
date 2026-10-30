// Copyright (c) 2026, Nisarg Panchal
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

#include <cmath>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "rclcpp/rclcpp.hpp"

constexpr unsigned int size = 60;
constexpr double resolution = 0.05;
constexpr unsigned int block_min = 10;
constexpr unsigned int block_max = 20;

// A block of inscribed cells with its lower-left corner cell left free.
// A point at 10.8 cells lies in that free corner cell (10, 10). Rounding it
// instead gives the inscribed cell (11, 11), which NavFn treats as lethal.
class NavfnCellTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("NavfnCellTest");
    // Zero tolerance so the planner cannot fall back to a nearby goal cell
    node_->declare_parameter("test.tolerance", 0.0);

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    auto costmap = costmap_ros_->getCostmap();
    costmap->resizeMap(size, size, resolution, 0.0, 0.0);
    for (unsigned int x = 0; x < size; ++x) {
      for (unsigned int y = 0; y < size; ++y) {
        const bool in_block = x >= block_min && x <= block_max &&
          y >= block_min && y <= block_max && !(x == block_min && y == block_min);
        costmap->setCost(
          x, y, in_block ?
          nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE : nav2_costmap_2d::FREE_SPACE);
      }
    }
    ASSERT_EQ(costmap->getCost(block_min, block_min), nav2_costmap_2d::FREE_SPACE);
    ASSERT_EQ(
      costmap->getCost(block_min + 1, block_min + 1),
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

    planner_ = std::make_unique<nav2_navfn_planner::NavfnPlanner>();
    auto tf = nav2::create_transform_buffer(node_);
    planner_->configure(node_, "test", tf, costmap_ros_);
    planner_->activate();
  }

  void TearDown() override
  {
    planner_->deactivate();
    planner_->cleanup();
  }

  static geometry_msgs::msg::PoseStamped makePose(double cell_x, double cell_y)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = cell_x * resolution;
    pose.pose.position.y = cell_y * resolution;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_navfn_planner::NavfnPlanner> planner_;
};

TEST_F(NavfnCellTest, testPlanFromStartNextToInscribedCells)
{
  const auto start = makePose(10.8, 10.8);
  const auto goal = makePose(50.0, 5.0);

  nav_msgs::msg::Path path;
  EXPECT_NO_THROW(path = planner_->createPlan(start, goal, {}, [] {return false;}));
  ASSERT_FALSE(path.poses.empty());

  // The path starts at the center of the start cell
  const double dx = path.poses.front().pose.position.x - start.pose.position.x;
  const double dy = path.poses.front().pose.position.y - start.pose.position.y;
  EXPECT_LE(std::hypot(dx, dy), std::sqrt(0.5) * resolution + 1e-6);
}

TEST_F(NavfnCellTest, testPlanToGoalNextToInscribedCells)
{
  const auto start = makePose(50.0, 5.0);
  const auto goal = makePose(10.8, 10.8);

  nav_msgs::msg::Path path;
  EXPECT_NO_THROW(path = planner_->createPlan(start, goal, {}, [] {return false;}));
  EXPECT_FALSE(path.poses.empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
