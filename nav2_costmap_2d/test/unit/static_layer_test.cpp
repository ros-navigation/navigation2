// Copyright (c) 2026 Open Navigation LLC
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
#include <string>

#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "tf2_ros/buffer.hpp"

class TestStaticLayer : public nav2_costmap_2d::StaticLayer
{
public:
  using StaticLayer::incomingMap;
};

class StaticLayerRollingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("static_layer_rolling_test");
    node_->declare_parameter("track_unknown_space", true);
    node_->declare_parameter("use_maximum", true);
    node_->declare_parameter("lethal_cost_threshold", 100);
    node_->declare_parameter("inscribed_obstacle_cost_value", 99);
    node_->declare_parameter("unknown_cost_value", 255);
    node_->declare_parameter("trinary_costmap", true);
    node_->declare_parameter("transform_tolerance", 0.0);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "map";
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf_->setTransform(transform, "test", true));
    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("odom", true, true);
    layers_->resizeMap(10, 10, 1.0, 0.0, 0.0);
    layer_ = std::make_shared<TestStaticLayer>();
    layers_->addPlugin(layer_);
    layer_->initialize(layers_.get(), "static", tf_.get(), node_, nullptr);
    layer_->activate();
  }

  void TearDown() override
  {
    layer_->deactivate();
    node_->shutdown();
  }

  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<TestStaticLayer> layer_;
};

TEST_F(StaticLayerRollingTest, MaximumMergeTreatsUnknownAsTransparent)
{
  // Map covers the window; one lethal cell, the rest unknown
  auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->header.frame_id = "map";
  map->info.width = map->info.height = 10;
  map->info.resolution = 1.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(100, -1);
  map->data[0] = 100;
  layer_->incomingMap(map);

  layers_->updateMap(5.0, 5.0, 0.0);
  auto * master = layers_->getCostmap();
  // Lethal must land on a master that starts unknown (255 > 254 under plain std::max)
  EXPECT_EQ(master->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(5, 5), nav2_costmap_2d::NO_INFORMATION);

  // Another layer marks a cell the map knows nothing about; the map must not erase it
  master->setCost(5, 5, nav2_costmap_2d::LETHAL_OBSTACLE);
  layer_->updateCosts(*master, 0, 0, 10, 10);
  EXPECT_EQ(master->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

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
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, true);
    layers_->resizeMap(20, 20, 1.0, 0.0, 0.0);
  }

  void TearDown() override
  {
    node_->shutdown();
  }

  std::shared_ptr<TestStaticLayer> addLayer(const std::string & name, bool resize_master)
  {
    node_->declare_parameter(name + ".resize_master", resize_master);
    auto layer = std::make_shared<TestStaticLayer>();
    layers_->addPlugin(layer);
    layer->initialize(layers_.get(), name, tf_.get(), node_, nullptr);
    layer->activate();
    return layer;
  }

  // 2x2 cells of 2 m: one lethal cell at (origin_x .. +2, 5 .. 7), rest unknown
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
};

TEST_F(StaticLayerOverlayTest, OverlayKeepsMasterGeometryAndProjectsCosts)
{
  auto overlay = addLayer("overlay", false);
  overlay->incomingMap(makeMap());
  auto * master = layers_->getCostmap();
  EXPECT_EQ(master->getSizeInCellsX(), 20u);
  EXPECT_EQ(master->getSizeInCellsY(), 20u);
  EXPECT_DOUBLE_EQ(master->getResolution(), 1.0);
  EXPECT_DOUBLE_EQ(master->getOriginX(), 0.0);

  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_EQ(master->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(6, 6), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(7, 5), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(master->getCost(0, 0), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_TRUE(overlay->isCurrent());
}

TEST_F(StaticLayerOverlayTest, ReplacementMapClearsThePreviousExtent)
{
  auto overlay = addLayer("overlay", false);
  overlay->incomingMap(makeMap());
  layers_->updateMap(10.0, 10.0, 0.0);
  ASSERT_EQ(layers_->getCostmap()->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);

  overlay->incomingMap(makeMap(10.0));
  layers_->updateMap(10.0, 10.0, 0.0);
  auto * master = layers_->getCostmap();
  EXPECT_EQ(master->getCost(10, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(5, 5), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(master->getSizeInCellsX(), 20u);
}

TEST_F(StaticLayerOverlayTest, TwoOverlaysShareOneMaster)
{
  auto first = addLayer("first", false);
  auto second = addLayer("second", false);
  first->incomingMap(makeMap(5.0));
  second->incomingMap(makeMap(10.0));
  layers_->updateMap(10.0, 10.0, 0.0);
  auto * master = layers_->getCostmap();
  EXPECT_EQ(master->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(10, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getSizeInCellsX(), 20u);
}

TEST_F(StaticLayerOverlayTest, DefaultStillResizesTheMaster)
{
  auto base = addLayer("static", true);
  base->incomingMap(makeMap());
  auto * master = layers_->getCostmap();
  EXPECT_EQ(master->getSizeInCellsX(), 2u);
  EXPECT_DOUBLE_EQ(master->getResolution(), 2.0);
  EXPECT_DOUBLE_EQ(master->getOriginX(), 5.0);
  layers_->updateMap(6.0, 6.0, 0.0);
  EXPECT_EQ(master->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(StaticLayerOverlayTest, ResizeMasterIsNotDynamic)
{
  auto overlay = addLayer("overlay", false);
  overlay->incomingMap(makeMap());
  // Like map_topic, the parameter is read at initialization only: setting it later has no effect
  node_->set_parameter(rclcpp::Parameter("overlay.resize_master", true));
  overlay->incomingMap(makeMap(10.0));
  layers_->updateMap(10.0, 10.0, 0.0);
  EXPECT_EQ(layers_->getCostmap()->getSizeInCellsX(), 20u);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
