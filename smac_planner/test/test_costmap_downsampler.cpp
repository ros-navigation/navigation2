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
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "smac_planner/costmap_downsampler.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestNode : public nav2_util::LifecycleNode
{
public:
  TestNode()
  : nav2_util::LifecycleNode("CostmapDownsamplerTest"),
    costmap_(nullptr),
    downsampler_(nullptr) {}

  void setCostmap(nav2_costmap_2d::Costmap2D * const costmap)
  {
    costmap_ = costmap;
  }

  std::shared_ptr<smac_planner::CostmapDownsampler> getDownsampler()
  {
    return downsampler_;
  }

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Configuring");
    downsampler_ = std::make_shared<smac_planner::CostmapDownsampler>();
    downsampler_->on_configure(shared_from_this(), "map", "unused_topic", costmap_, 2);
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Activating");
    downsampler_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating");
    downsampler_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");
    downsampler_->on_cleanup();
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<smac_planner::CostmapDownsampler> downsampler_;
};

TEST(CostmapDownsampler, costmap_downsample_test)
{
  auto node = std::make_shared<TestNode>();

  // create basic costmap
  nav2_costmap_2d::Costmap2D costmapA(10, 10, 0.05, 0.0, 0.0, 0);
  costmapA.setCost(0, 0, 100);
  costmapA.setCost(5, 5, 50);

  // set costmap for testing
  node->setCostmap(&costmapA);

  // configure and activate test node
  node->configure();
  node->activate();

  // get downsampler
  auto downsampler = node->getDownsampler();

  // downsample it
  nav2_costmap_2d::Costmap2D * downsampledCostmapA = downsampler->downsample(2);

  // validate it
  EXPECT_EQ(downsampledCostmapA->getCost(0, 0), 100);
  EXPECT_EQ(downsampledCostmapA->getCost(2, 2), 50);
  EXPECT_EQ(downsampledCostmapA->getSizeInCellsX(), 5u);
  EXPECT_EQ(downsampledCostmapA->getSizeInCellsY(), 5u);

  // deactivate and cleanup test node
  node->deactivate();
  node->cleanup();

  // give it another costmap of another size
  nav2_costmap_2d::Costmap2D costmapB(4, 4, 0.10, 0.0, 0.0, 0);

  // set costmap for testing
  node->setCostmap(&costmapB);

  // configure and activate test node
  node->configure();
  node->activate();

  // get downsampler
  downsampler = node->getDownsampler();

  // downsample it
  nav2_costmap_2d::Costmap2D * downsampledCostmapB = downsampler->downsample(4);

  // validate size
  EXPECT_EQ(downsampledCostmapB->getSizeInCellsX(), 1u);
  EXPECT_EQ(downsampledCostmapB->getSizeInCellsY(), 1u);

  downsampler->resizeCostmap();

  // deactivate and cleanup test node
  node->deactivate();
  node->cleanup();
  node->shutdown();
}
