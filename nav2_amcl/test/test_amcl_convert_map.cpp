// Copyright (c) 2025
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

#include "gtest/gtest.h"
#include "nav2_amcl/amcl_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>


class AmclNodeTest : public ::nav2_amcl::AmclNode, public ::testing::Test
{
public:
  AmclNodeTest()
  : nav2_amcl::AmclNode(rclcpp::NodeOptions())
  {
  }
  void SetUp() override
  {
  }

  void TearDown() override
  {
  }

  nav_msgs::msg::OccupancyGrid createOccupancyGrid(
    int width, int height, double resolution,
    const std::string & frame_id)
  {
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header.frame_id = frame_id;
    occupancy_grid.info.width = width;
    occupancy_grid.info.height = height;
    occupancy_grid.info.resolution = resolution;
    occupancy_grid.info.origin.orientation.w = 1.0;
    occupancy_grid.data.assign(static_cast<size_t>(width) * static_cast<size_t>(height), 0);
    return occupancy_grid;
  }
};

TEST_F(AmclNodeTest, test_convert_oversized_map)
{
  // 65536 × 65536 = 4,294,967,296 = 0x100000000 (overflows to 0 in uint32_t)
  // map width and width is uint32
  auto map = createOccupancyGrid(65536, 65536, 0.05, "map");
  auto converted_map = this->convertMap(map);
  EXPECT_EQ(converted_map, nullptr);
}

TEST_F(AmclNodeTest, test_close_to_oversized_map)
{
  // 65536 × 65536 = 4,294,967,296 = 0x100000000 (overflows to 0 in uint32_t)
  // map width and width is uint32
  auto map = createOccupancyGrid(INT16_MAX, INT16_MAX, 0.05, "map");
  auto converted_map = this->convertMap(map);
  EXPECT_NE(converted_map, nullptr);
}

TEST_F(AmclNodeTest, test_convert_close_oversized_map)
{
  auto map = createOccupancyGrid(20, 20, 0.05, "map");
  auto converted_map = this->convertMap(map);
  EXPECT_NE(converted_map, nullptr);
}

TEST_F(AmclNodeTest, test_empty_data)
{
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header.frame_id = "map";
  occupancy_grid.info.width = 20;
  occupancy_grid.info.height = 20;
  occupancy_grid.info.resolution = 0.05;
  occupancy_grid.info.origin.orientation.w = 1.0;
  auto converted_map = this->convertMap(occupancy_grid);
  EXPECT_EQ(converted_map, nullptr);
}

TEST_F(AmclNodeTest, test_overflow_int_width)
{
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header.frame_id = "map";
  occupancy_grid.info.width = INT16_MAX + 1;
  occupancy_grid.info.height = 20;
  occupancy_grid.info.resolution = 0.05;
  occupancy_grid.info.origin.orientation.w = 1.0;
  occupancy_grid.data.assign(
    static_cast<size_t>(occupancy_grid.info.width) *
    static_cast<size_t>(occupancy_grid.info.height), 0);
  auto converted_map = this->convertMap(occupancy_grid);
  EXPECT_EQ(converted_map, nullptr);
}

TEST_F(AmclNodeTest, test_overflow_int_height)
{
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header.frame_id = "map";
  occupancy_grid.info.width = 20;
  occupancy_grid.info.height = INT16_MAX + 1;
  occupancy_grid.info.resolution = 0.05;
  occupancy_grid.info.origin.orientation.w = 1.0;
  occupancy_grid.data.assign(
    static_cast<size_t>(occupancy_grid.info.width) *
    static_cast<size_t>(occupancy_grid.info.height), 0);
  auto converted_map = this->convertMap(occupancy_grid);
  EXPECT_EQ(converted_map, nullptr);
}

int main(int argc, char ** argv)
{
  // Initialize rclcpp before creating any nodes (nodes/guard conditions expect a valid context)
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  // Shutdown rclcpp to clean up context
  rclcpp::shutdown();
  return result;
}
