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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
