// Copyright (c) 2026 Marc Blöchlinger
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

// Integration tests for AsymmetricInflationLayer.
//
// These tests stand up a minimal LayeredCostmap with the asymmetric layer
// attached and exercise updateCosts() on synthetic obstacle layouts.

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/inflation_layer_interface.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/asymmetric_inflation_layer.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using geometry_msgs::msg::Point;

class TestableAsymmetricInflationLayer : public nav2_costmap_2d::AsymmetricInflationLayer
{
public:
  void injectPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    globalPathCallback(msg);
  }

  void setSideFactors(double left, double right)
  {
    cost_scaling_factor_left_ = left;
    cost_scaling_factor_right_ = right;
    cost_scaling_factor_ = std::max(left, right);
    need_reinflation_ = true;
    setCurrent(false);
    matchSize();
  }
};

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcpp_fixture;

class AsymmetricInflationIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Lifecycle node with the parameters the layer expects.
    auto options = rclcpp::NodeOptions();
    options.parameter_overrides(
    {
      {"asymmetric_inflation_layer.enabled", true},
      {"asymmetric_inflation_layer.inflation_radius", 0.55},
      {"asymmetric_inflation_layer.cost_scaling_factor_left", 10.0},
      {"asymmetric_inflation_layer.cost_scaling_factor_right", 10.0},
      {"asymmetric_inflation_layer.inflate_unknown", false},
      {"asymmetric_inflation_layer.inflate_around_unknown", false},
      {"asymmetric_inflation_layer.plan_topic", std::string("plan")},
      {"asymmetric_inflation_layer.goal_distance_threshold", 1.5},
      {"map_topic", std::string("map")},
      {"track_unknown_space", false},
      {"use_maximum", false},
      {"lethal_cost_threshold", 100},
      {"unknown_cost_value", static_cast<unsigned char>(0xff)},
      {"trinary_costmap", true},
      {"transform_tolerance", 0.3},
      {"observation_sources", std::string("")},
    });

    node_ = std::make_shared<nav2::LifecycleNode>(
      "asymmetric_inflation_test_node", "", options);

    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    // 40x40 cells, 0.1m resolution, origin at (0,0) — world range [0, 4]m.
    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    layers_->resizeMap(40, 40, 0.1, 0.0, 0.0);

    // Give the layered costmap a small square footprint — inscribed ≈ 0.1m.
    std::vector<Point> footprint;
    Point p;
    p.x = 0.1; p.y = 0.1; footprint.push_back(p);
    p.x = 0.1; p.y = -0.1; footprint.push_back(p);
    p.x = -0.1; p.y = -0.1; footprint.push_back(p);
    p.x = -0.1; p.y = 0.1; footprint.push_back(p);
    layers_->setFootprint(footprint);

    layer_ = std::make_shared<TestableAsymmetricInflationLayer>();
    layer_->initialize(
      layers_.get(), "asymmetric_inflation_layer", tf_.get(), node_, nullptr);
    layers_->addPlugin(std::static_pointer_cast<nav2_costmap_2d::Layer>(layer_));
  }

  void seedLethalObstacle(unsigned int mx, unsigned int my)
  {
    layers_->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  void runInflation()
  {
    // Run the standalone layer on the full map.
    auto * costmap = layers_->getCostmap();
    unsigned int w = costmap->getSizeInCellsX();
    unsigned int h = costmap->getSizeInCellsY();
    layer_->updateCosts(*costmap, 0, 0, w, h);
  }

  void injectStraightPath()
  {
    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header.frame_id = "map";

    for (double x : {0.5, 1.5, 2.5, 3.5}) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = x;
      pose.pose.position.y = 2.0;
      path->poses.push_back(pose);
    }

    layer_->injectPath(path);
  }

  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<TestableAsymmetricInflationLayer> layer_;
};

// Test: without a published path the layer must behave symmetrically
// (the no-asymmetry short-circuit in updateCosts skips the BFS because
// extractLocalPath returns empty).
TEST_F(AsymmetricInflationIntegrationTest, no_path_produces_symmetric_output)
{
  seedLethalObstacle(20, 20);

  // Kick the layer — no path has been published.
  runInflation();

  auto * costmap = layers_->getCostmap();
  // With no path, the two horizontally-mirrored cells around the obstacle must
  // receive identical costs (Pass 1 only, which is symmetric).
  unsigned char left = costmap->getCost(15, 20);
  unsigned char right = costmap->getCost(25, 20);
  EXPECT_EQ(left, right) << "Without a path, costs must be symmetric";
  EXPECT_GT(left, nav2_costmap_2d::FREE_SPACE)
    << "Standalone asymmetric layer must still write the symmetric baseline";
}

// Test: with cost_scaling_factor_left == cost_scaling_factor_right the layer
// is a no-op overlay even when a path is available, so the costmap stays symmetric.
TEST_F(AsymmetricInflationIntegrationTest, equal_sides_produce_symmetric_output)
{
  // Both sides set to 10.0 in SetUp() — equal decay rates short-circuit the BFS.
  injectStraightPath();
  seedLethalObstacle(20, 20);
  seedLethalObstacle(20, 30);  // second obstacle so we can also check vertical symmetry

  runInflation();

  auto * costmap = layers_->getCostmap();
  // Horizontal symmetry across x = 20
  for (int dy = -5; dy <= 5; ++dy) {
    unsigned char left = costmap->getCost(15, 20 + dy);
    unsigned char right = costmap->getCost(25, 20 + dy);
    EXPECT_EQ(left, right) << "Asymmetry with equal sides (y_off=" << dy << ")";
  }
}

TEST_F(AsymmetricInflationIntegrationTest, unequal_sides_raise_costs_from_standalone_layer)
{
  layer_->setSideFactors(1.0, 10.0);
  injectStraightPath();

  seedLethalObstacle(20, 24);  // left/north of the path, within inflation radius
  seedLethalObstacle(20, 16);  // right/south of the path, within inflation radius

  runInflation();

  auto * costmap = layers_->getCostmap();
  unsigned char left_side_cost = costmap->getCost(20, 28);
  unsigned char right_side_cost = costmap->getCost(20, 12);
  EXPECT_GT(left_side_cost, right_side_cost)
    << "The smaller left-side decay factor should raise costs above the symmetric baseline";
}

TEST_F(AsymmetricInflationIntegrationTest, implements_inflation_layer_interface)
{
  std::shared_ptr<nav2_costmap_2d::Layer> base_layer = layer_;
  auto inflation_layer =
    std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayerInterface>(base_layer);

  ASSERT_NE(inflation_layer, nullptr);
  EXPECT_DOUBLE_EQ(inflation_layer->getInflationRadius(), 0.55);
  EXPECT_DOUBLE_EQ(inflation_layer->getCostScalingFactor(), 10.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
