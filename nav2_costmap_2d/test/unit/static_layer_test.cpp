// Copyright (c) 2026 Dexory
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

#include <gtest/gtest.h>

#include <memory>

#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "tf2_ros/buffer.hpp"

class TestStaticLayer : public nav2_costmap_2d::StaticLayer
{
public:
  using StaticLayer::incomingMap;
  using StaticLayer::incomingUpdate;
  using StaticLayer::incomingSourceReady;

  void setSourceReady(bool ready)
  {
    auto msg = std::make_shared<std_msgs::msg::Bool>();
    msg->data = ready;
    incomingSourceReady(msg);
  }
};

class StaticLayerOverlayTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("static_layer_overlay_test");
    node_->declare_parameter("track_unknown_space", true);
    node_->declare_parameter("use_maximum", true);
    node_->declare_parameter("lethal_cost_threshold", 100);
    node_->declare_parameter("inscribed_obstacle_cost_value", 99);
    node_->declare_parameter("unknown_cost_value", 255);
    node_->declare_parameter("trinary_costmap", true);
    node_->declare_parameter("transform_tolerance", 0.0);
    node_->declare_parameter("overlay.resize_master", false);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, true);
    layers_->resizeMap(20, 20, 1.0, 0.0, 0.0);
    overlay_ = std::make_shared<TestStaticLayer>();
    layers_->addPlugin(overlay_);
    overlay_->initialize(layers_.get(), "overlay", tf_.get(), node_, nullptr);
    overlay_->activate();
  }

  void TearDown() override
  {
    overlay_->deactivate();
    node_->shutdown();
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr makeMap(double origin_x = 5.0)
  {
    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->header.frame_id = "map";
    map->info.width = 2;
    map->info.height = 2;
    map->info.resolution = 2.0;
    map->info.origin.position.x = origin_x;
    map->info.origin.position.y = 5.0;
    map->info.origin.orientation.w = 1.0;
    map->data = {100, -1, -1, -1};
    return map;
  }

  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<TestStaticLayer> overlay_;
};

TEST_F(StaticLayerOverlayTest, PreservesMasterGeometryAndProjectsCosts)
{
  overlay_->incomingMap(makeMap());
  auto * master = layers_->getCostmap();
  ASSERT_EQ(master->getSizeInCellsX(), 20u);
  ASSERT_EQ(master->getSizeInCellsY(), 20u);
  ASSERT_DOUBLE_EQ(master->getResolution(), 1.0);
  ASSERT_DOUBLE_EQ(master->getOriginX(), 0.0);
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_EQ(master->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(6, 6), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(7, 5), nav2_costmap_2d::NO_INFORMATION);
}

TEST_F(StaticLayerOverlayTest, ReplacementAndEmptyMapClearPreviousExtent)
{
  overlay_->incomingMap(makeMap());
  layers_->updateMap(10.0, 10.0, 0.0);
  overlay_->incomingMap(makeMap(10.0));
  layers_->updateMap(10.0, 10.0, 0.0);
  auto * master = layers_->getCostmap();
  EXPECT_EQ(master->getCost(5, 5), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(master->getCost(10, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  auto empty_map = makeMap(0.0);
  empty_map->info.width = empty_map->info.height = 1;
  empty_map->data = {-1};
  overlay_->incomingMap(empty_map);
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_EQ(master->getCost(10, 5), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(master->getSizeInCellsX(), 20u);
}

TEST_F(StaticLayerOverlayTest, DefaultBaseLayerResizesAndSurvivesOverlayRemoval)
{
  overlay_->deactivate();
  layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, true);
  auto base = std::make_shared<TestStaticLayer>();
  layers_->addPlugin(base);
  base->initialize(layers_.get(), "base", tf_.get(), node_, nullptr);
  auto base_map = makeMap(0.0);
  base_map->info.width = base_map->info.height = 20;
  base_map->info.resolution = 1.0;
  base_map->info.origin.position.y = 0.0;
  base_map->data.assign(400, 0);
  base_map->data[5 * 20 + 7] = 100;
  base->incomingMap(base_map);
  ASSERT_EQ(layers_->getCostmap()->getSizeInCellsX(), 20u);
  overlay_ = std::make_shared<TestStaticLayer>();
  layers_->addPlugin(overlay_);
  overlay_->initialize(layers_.get(), "overlay", tf_.get(), node_, nullptr);
  overlay_->incomingMap(makeMap());
  layers_->updateMap(10.0, 10.0, 0.0);
  auto * master = layers_->getCostmap();
  EXPECT_EQ(master->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(7, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(8, 5), nav2_costmap_2d::FREE_SPACE);
  auto map = makeMap();
  map->data.assign(4, -1);
  overlay_->incomingMap(map);
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_EQ(master->getCost(5, 5), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(master->getCost(7, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(StaticLayerOverlayTest, DisableClearsAndEnableRestores)
{
  overlay_->incomingMap(makeMap());
  layers_->updateMap(10.0, 10.0, 0.0);
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("overlay.enabled", false)).successful);
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_EQ(layers_->getCostmap()->getCost(5, 5), nav2_costmap_2d::NO_INFORMATION);
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("overlay.enabled", true)).successful);
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_EQ(layers_->getCostmap()->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("overlay.resize_master", true)).successful);
}

TEST_F(StaticLayerOverlayTest, TransformMovementClearsOldCostsWithoutNewMessage)
{
  auto map = makeMap();
  map->header.frame_id = "moving_map";
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "moving_map";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_->setTransform(transform, "test", true));
  overlay_->incomingMap(map);
  layers_->updateMap(10.0, 10.0, 0.0);
  ASSERT_EQ(layers_->getCostmap()->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  transform.transform.translation.x = 5.0;
  ASSERT_TRUE(tf_->setTransform(transform, "test", true));
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_EQ(layers_->getCostmap()->getCost(5, 5), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(layers_->getCostmap()->getCost(10, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(StaticLayerOverlayTest, MissingTransformRetriesWithoutClaimingCurrent)
{
  auto map = makeMap();
  map->header.frame_id = "missing_map";
  overlay_->incomingMap(map);
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_FALSE(overlay_->isCurrent());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "missing_map";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_->setTransform(transform, "test", true));
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_TRUE(overlay_->isCurrent());
  EXPECT_EQ(layers_->getCostmap()->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(StaticLayerOverlayTest, MasterResizeRetainsOverlayData)
{
  overlay_->incomingMap(makeMap());
  layers_->updateMap(10.0, 10.0, 0.0);
  layers_->resizeMap(30, 30, 1.0, 0.0, 0.0);
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_EQ(overlay_->getSizeInCellsX(), 2u);
  EXPECT_EQ(layers_->getCostmap()->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(StaticLayerOverlayTest, OverlaysInflateAndRemovalClearsInflation)
{
  node_->declare_parameter("inflation.inflation_radius", 3.0);
  node_->declare_parameter("inflation.cost_scaling_factor", 1.0);
  node_->declare_parameter("inflation.inflate_unknown", true);
  auto inflation = std::make_shared<nav2_costmap_2d::InflationLayer>();
  layers_->addPlugin(inflation);
  inflation->initialize(layers_.get(), "inflation", tf_.get(), node_, nullptr);
  overlay_->incomingMap(makeMap());
  layers_->updateMap(10.0, 10.0, 0.0);
  const auto inflated_cost = layers_->getCostmap()->getCost(4, 5);
  EXPECT_GT(inflated_cost, nav2_costmap_2d::FREE_SPACE);
  EXPECT_LT(inflated_cost, nav2_costmap_2d::LETHAL_OBSTACLE);
  auto map = makeMap();
  map->data.assign(4, -1);
  overlay_->incomingMap(map);
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_EQ(layers_->getCostmap()->getCost(4, 5), nav2_costmap_2d::NO_INFORMATION);
}

TEST_F(StaticLayerOverlayTest, RollingWindowProjectsOverlayAfterOriginShift)
{
  overlay_->deactivate();
  layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("odom", true, true);
  layers_->resizeMap(10, 10, 1.0, 0.0, 0.0);
  overlay_ = std::make_shared<TestStaticLayer>();
  layers_->addPlugin(overlay_);
  overlay_->initialize(layers_.get(), "overlay", tf_.get(), node_, nullptr);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "map";
  transform.transform.rotation.w = 1.0;
  transform.transform.translation.x = 1.0;
  ASSERT_TRUE(tf_->setTransform(transform, "test", true));
  overlay_->incomingMap(makeMap());
  layers_->updateMap(5.0, 5.0, 0.0);
  layers_->updateMap(8.0, 5.0, 0.0);
  auto * master = layers_->getCostmap();
  unsigned int cell_x, cell_y;
  ASSERT_TRUE(master->worldToMap(6.5, 5.5, cell_x, cell_y));
  EXPECT_EQ(master->getCost(cell_x, cell_y), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getSizeInCellsX(), 10u);
}

TEST_F(StaticLayerOverlayTest, RollingDefaultLayerKeepsLethalUnderUnknownMapCells)
{
  // A rolling costmap with the stock static layer (resize_master left at its default)
  overlay_->deactivate();
  layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("odom", true, true);
  layers_->resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto base = std::make_shared<TestStaticLayer>();
  layers_->addPlugin(base);
  base->initialize(layers_.get(), "base", tf_.get(), node_, nullptr);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "map";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_->setTransform(transform, "test", true));
  // Map covers the whole window; only one cell is lethal, the rest unknown
  auto map = makeMap(0.0);
  map->info.width = map->info.height = 5;
  map->info.origin.position.y = 0.0;
  map->data.assign(25, -1);
  map->data[0] = 100;
  base->incomingMap(map);
  layers_->updateMap(5.0, 5.0, 0.0);
  auto * master = layers_->getCostmap();
  EXPECT_EQ(master->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(5, 5), nav2_costmap_2d::NO_INFORMATION);

  // Another layer marks a cell the map knows nothing about; the map must not erase it
  master->setCost(5, 5, nav2_costmap_2d::LETHAL_OBSTACLE);
  base->updateCosts(*master, 0, 0, 10, 10);
  EXPECT_EQ(master->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(StaticLayerOverlayTest, SourceNotReadyHoldsNotCurrentUntilNewMapApplied)
{
  overlay_->incomingMap(makeMap());
  layers_->updateMap(10.0, 10.0, 0.0);
  ASSERT_TRUE(overlay_->isCurrent());

  overlay_->setSourceReady(false);
  EXPECT_FALSE(overlay_->isCurrent());
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_FALSE(overlay_->isCurrent());

  // Ready arrives before the map it announces
  overlay_->setSourceReady(true);
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_FALSE(overlay_->isCurrent());

  overlay_->incomingMap(makeMap(10.0));
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_TRUE(overlay_->isCurrent());
  EXPECT_EQ(layers_->getCostmap()->getCost(10, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(StaticLayerOverlayTest, MapBeforeSourceReadyBecomesCurrentOnReady)
{
  overlay_->incomingMap(makeMap());
  layers_->updateMap(10.0, 10.0, 0.0);
  overlay_->setSourceReady(false);
  overlay_->incomingMap(makeMap(10.0));
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_FALSE(overlay_->isCurrent());

  // No further update cycle is needed once the source confirms
  overlay_->setSourceReady(true);
  EXPECT_TRUE(overlay_->isCurrent());
}

TEST_F(StaticLayerOverlayTest, SourceReadyWithoutAnyMapStaysNotCurrent)
{
  overlay_->setSourceReady(false);
  overlay_->setSourceReady(true);
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_FALSE(overlay_->isCurrent());

  overlay_->incomingMap(makeMap());
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_TRUE(overlay_->isCurrent());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
