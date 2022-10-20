// Copyright (c) 2020 Samsung Research Russia
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

#include <vector>
#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

static constexpr double EPSILON = std::numeric_limits<float>::epsilon();
static constexpr double RESOLUTION = 0.05;
static constexpr double ORIGIN_X = 0.1;
static constexpr double ORIGIN_Y = 0.2;

class TestNode : public ::testing::Test
{
public:
  TestNode() {}

  ~TestNode()
  {
    occ_grid_.reset();
    costmap_.reset();
  }

protected:
  void createMaps();
  void verifyCostmap();

private:
  std::shared_ptr<nav_msgs::msg::OccupancyGrid> occ_grid_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
};

void TestNode::createMaps()
{
  // Create occ_grid_ map
  occ_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

  const unsigned int width = 4;
  const unsigned int height = 3;

  occ_grid_->info.resolution = RESOLUTION;
  occ_grid_->info.width = width;
  occ_grid_->info.height = height;
  occ_grid_->info.origin.position.x = ORIGIN_X;
  occ_grid_->info.origin.position.y = ORIGIN_Y;
  occ_grid_->info.origin.position.z = 0.0;
  occ_grid_->info.origin.orientation.x = 0.0;
  occ_grid_->info.origin.orientation.y = 0.0;
  occ_grid_->info.origin.orientation.z = 0.0;
  occ_grid_->info.origin.orientation.w = 1.0;
  occ_grid_->data.resize(width * height);

  int8_t data;
  for (unsigned int i = 0; i < width * height; i++) {
    data = i * 10;
    if (data <= nav2_util::OCC_GRID_OCCUPIED) {
      occ_grid_->data[i] = data;
    } else {
      occ_grid_->data[i] = nav2_util::OCC_GRID_UNKNOWN;
    }
  }

  // Create costmap_ (convert OccupancyGrid -> to Costmap2D)
  costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(*occ_grid_);
}

void TestNode::verifyCostmap()
{
  // Verify Costmap2D info
  EXPECT_NEAR(costmap_->getResolution(), RESOLUTION, EPSILON);
  EXPECT_NEAR(costmap_->getOriginX(), ORIGIN_X, EPSILON);
  EXPECT_NEAR(costmap_->getOriginY(), ORIGIN_Y, EPSILON);

  // Verify Costmap2D data
  unsigned int it;
  unsigned char data, data_ref;
  for (it = 0; it < (costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY() - 1); it++) {
    data = costmap_->getCharMap()[it];
    if (it != costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY() - 1) {
      data_ref = std::round(
        static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE - nav2_costmap_2d::FREE_SPACE) * it /
        10);
    } else {
      data_ref = nav2_costmap_2d::NO_INFORMATION;
    }
    EXPECT_EQ(data, data_ref);
  }
}

TEST_F(TestNode, convertOccGridToCostmap)
{
  createMaps();
  verifyCostmap();
}
