// Copyright (c) 2023 Samsung R&D Institute Russia
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

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

class CostmapFilterWrapper : public nav2_costmap_2d::CostmapFilter
{
public:
  CostmapFilterWrapper() {}

  bool worldToMask(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask,
    double wx, double wy, unsigned int & mx, unsigned int & my) const
  {
    return nav2_costmap_2d::CostmapFilter::worldToMask(filter_mask, wx, wy, mx, my);
  }

  unsigned char getMaskCost(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask,
    const unsigned int mx, const unsigned int & my) const
  {
    return nav2_costmap_2d::CostmapFilter::getMaskCost(filter_mask, mx, my);
  }

  // API coverage
  void initializeFilter(const std::string &) {}
  void process(
    nav2_costmap_2d::Costmap2D &, int, int, int, int, const geometry_msgs::msg::Pose2D &)
  {}
  void resetFilter() {}
};

TEST(CostmapFilter, testWorldToMask)
{
  // Create occupancy grid for test as follows:
  //
  //   ^
  //   |        (6,6)
  //   |    *-----*
  //   |    |/////| <- mask
  //   |    |/////|
  //   |    *-----*
  //   |  (3,3)
  //   *---------------->
  // (0,0)

  const unsigned int width = 3;
  const unsigned int height = 3;

  auto mask = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  mask->header.frame_id = "map";
  mask->info.resolution = 1.0;
  mask->info.width = width;
  mask->info.height = height;
  mask->info.origin.position.x = 3.0;
  mask->info.origin.position.y = 3.0;

  mask->data.resize(width * height, nav2_util::OCC_GRID_OCCUPIED);

  CostmapFilterWrapper cf;
  unsigned int mx, my;
  // Point inside mask
  ASSERT_TRUE(cf.worldToMask(mask, 4.0, 5.0, mx, my));
  ASSERT_EQ(mx, 1u);
  ASSERT_EQ(my, 2u);
  // Corner cases
  ASSERT_TRUE(cf.worldToMask(mask, 3.0, 3.0, mx, my));
  ASSERT_EQ(mx, 0u);
  ASSERT_EQ(my, 0u);
  ASSERT_TRUE(cf.worldToMask(mask, 5.9, 5.9, mx, my));
  ASSERT_EQ(mx, 2u);
  ASSERT_EQ(my, 2u);
  // Point outside mask
  ASSERT_FALSE(cf.worldToMask(mask, 2.9, 2.9, mx, my));
  ASSERT_FALSE(cf.worldToMask(mask, 6.0, 6.0, mx, my));
}

TEST(CostmapFilter, testGetMaskCost)
{
  // Create occupancy grid for test as follows:
  // [-1, 0,
  //  50, 100]

  const unsigned int width = 2;
  const unsigned int height = 2;

  auto mask = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  mask->header.frame_id = "map";
  mask->info.resolution = 1.0;
  mask->info.width = width;
  mask->info.height = height;
  mask->info.origin.position.x = 0.0;
  mask->info.origin.position.y = 0.0;

  mask->data.resize(width * height);
  mask->data[0] = nav2_util::OCC_GRID_UNKNOWN;
  mask->data[1] = nav2_util::OCC_GRID_FREE;
  mask->data[2] = nav2_util::OCC_GRID_OCCUPIED / 2;
  mask->data[3] = nav2_util::OCC_GRID_OCCUPIED;

  CostmapFilterWrapper cf;

  // Test all value cases
  ASSERT_EQ(cf.getMaskCost(mask, 0, 0), nav2_costmap_2d::NO_INFORMATION);
  ASSERT_EQ(cf.getMaskCost(mask, 1, 0), nav2_costmap_2d::FREE_SPACE);
  ASSERT_EQ(cf.getMaskCost(mask, 0, 1), nav2_costmap_2d::LETHAL_OBSTACLE / 2);
  ASSERT_EQ(cf.getMaskCost(mask, 1, 1), nav2_costmap_2d::LETHAL_OBSTACLE);
}

int main(int argc, char ** argv)
{
  // Initialize the system
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Actual testing
  bool test_result = RUN_ALL_TESTS();

  // Shutdown
  rclcpp::shutdown();

  return test_result;
}
