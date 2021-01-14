// Copyright (c) 2020, Samsung Research America
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
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_smac_planner/costmap_downsampler.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(CostmapDownsampler, costmap_downsample_test)
{
  nav2_util::LifecycleNode::SharedPtr node = std::make_shared<nav2_util::LifecycleNode>(
    "CostmapDownsamplerTest");
  nav2_smac_planner::CostmapDownsampler downsampler;

  // create basic costmap
  nav2_costmap_2d::Costmap2D costmapA(10, 10, 0.05, 0.0, 0.0, 0);
  costmapA.setCost(0, 0, 100);
  costmapA.setCost(5, 5, 50);

  // downsample it
  downsampler.on_configure(node, "map", "unused_topic", &costmapA, 2);
  nav2_costmap_2d::Costmap2D * downsampledCostmapA = downsampler.downsample(2);

  // validate it
  EXPECT_EQ(downsampledCostmapA->getCost(0, 0), 100);
  EXPECT_EQ(downsampledCostmapA->getCost(2, 2), 50);
  EXPECT_EQ(downsampledCostmapA->getSizeInCellsX(), 5u);
  EXPECT_EQ(downsampledCostmapA->getSizeInCellsY(), 5u);

  // give it another costmap of another size
  nav2_costmap_2d::Costmap2D costmapB(4, 4, 0.10, 0.0, 0.0, 0);

  // downsample it
  downsampler.on_configure(node, "map", "unused_topic", &costmapB, 4);
  downsampler.on_activate();
  nav2_costmap_2d::Costmap2D * downsampledCostmapB = downsampler.downsample(4);
  downsampler.on_deactivate();

  // validate size
  EXPECT_EQ(downsampledCostmapB->getSizeInCellsX(), 1u);
  EXPECT_EQ(downsampledCostmapB->getSizeInCellsY(), 1u);

  downsampler.resizeCostmap();
}
