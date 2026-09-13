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

#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.hpp"

class TestStaticLayer : public nav2_costmap_2d::StaticLayer
{
public:
  using StaticLayer::incomingMap;
  using StaticLayer::incomingSourceBarrier;
};

class StaticLayerSourceBarrierTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("static_layer_source_barrier_test");
    node_->declare_parameter("track_unknown_space", true);
    node_->declare_parameter("use_maximum", true);
    node_->declare_parameter("lethal_cost_threshold", 100);
    node_->declare_parameter("inscribed_obstacle_cost_value", 99);
    node_->declare_parameter("unknown_cost_value", 255);
    node_->declare_parameter("trinary_costmap", true);
    node_->declare_parameter("transform_tolerance", 0.0);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, true);
    layers_->resizeMap(10, 10, 1.0, 0.0, 0.0);
    layer_ = std::make_shared<TestStaticLayer>();
    layers_->addPlugin(layer_);
    layer_->initialize(layers_.get(), "static", tf_.get(), node_, nullptr);
  }

  void TearDown() override
  {
    node_->shutdown();
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr stampedMap(int32_t seconds)
  {
    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->header.frame_id = "map";
    map->header.stamp.sec = seconds;
    map->info.width = 10;
    map->info.height = 10;
    map->info.resolution = 1.0;
    map->info.origin.orientation.w = 1.0;
    map->data.assign(100, 0);
    map->data[0] = 100;
    return map;
  }

  std_msgs::msg::Header::SharedPtr barrier(int32_t seconds)
  {
    auto header = std::make_shared<std_msgs::msg::Header>();
    header->stamp.sec = seconds;
    return header;
  }

  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<TestStaticLayer> layer_;
};

TEST_F(StaticLayerSourceBarrierTest, IsCurrentWithoutABarrier)
{
  EXPECT_EQ(node_->get_parameter("static.source_ready_topic").as_string(), "");
  layer_->incomingMap(stampedMap(100));
  layers_->updateMap(5.0, 5.0, 0.0);
  EXPECT_TRUE(layer_->isCurrent());
}

TEST_F(StaticLayerSourceBarrierTest, WithholdsCurrencyUntilAMapSatisfiesTheBarrier)
{
  layer_->incomingMap(stampedMap(100));
  layers_->updateMap(5.0, 5.0, 0.0);
  ASSERT_TRUE(layer_->isCurrent());

  layer_->incomingSourceBarrier(barrier(200));
  EXPECT_FALSE(layer_->isCurrent());
  layers_->updateMap(5.0, 5.0, 0.0);
  EXPECT_FALSE(layer_->isCurrent());

  // A map that still predates the barrier does not satisfy it
  layer_->incomingMap(stampedMap(150));
  layers_->updateMap(5.0, 5.0, 0.0);
  EXPECT_FALSE(layer_->isCurrent());

  layer_->incomingMap(stampedMap(250));
  layers_->updateMap(5.0, 5.0, 0.0);
  EXPECT_TRUE(layer_->isCurrent());
}

TEST_F(StaticLayerSourceBarrierTest, ClearedBarrierRestoresCurrencyWithoutANewMap)
{
  layer_->incomingMap(stampedMap(100));
  layers_->updateMap(5.0, 5.0, 0.0);
  layer_->incomingSourceBarrier(barrier(200));
  ASSERT_FALSE(layer_->isCurrent());

  // A settled layer reports no bounds, so the zero stamp has to force an update pass
  layer_->incomingSourceBarrier(barrier(0));
  layers_->updateMap(5.0, 5.0, 0.0);
  EXPECT_TRUE(layer_->isCurrent());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
