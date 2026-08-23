// Copyright (c) 2026, LonelyGuy-SE1
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

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/fence_layer.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace nav2_costmap_2d
{

class TestLifecycleNode : public nav2::LifecycleNode
{
public:
  explicit TestLifecycleNode(const std::string & name)
  : nav2::LifecycleNode(name)
  {
  }

  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn onError(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }
};

class FenceLayerTester
{
public:
  static void setPolygonDirect(
    FenceLayer & layer,
    const std::vector<double> & x,
    const std::vector<double> & y)
  {
    layer.polygon_x_ = x;
    layer.polygon_y_ = y;
    layer.has_received_polygon_ = true;
  }

  static bool isPointInPolygon(const FenceLayer & layer, double x, double y)
  {
    return layer.isPointInPolygon(x, y);
  }

  static void triggerPolygonCallback(
    FenceLayer & layer,
    const geometry_msgs::msg::PolygonStamped::ConstSharedPtr & msg)
  {
    layer.polygonCallback(msg);
  }

  static bool hasReceivedPolygon(const FenceLayer & layer)
  {
    return layer.has_received_polygon_;
  }

  static bool isPolygonUpdated(const FenceLayer & layer)
  {
    return layer.polygon_updated_.load();
  }

  static const std::vector<bool> & getOutsideMask(const FenceLayer & layer)
  {
    return layer.outside_fence_;
  }

  static void getFenceDimensions(
    const FenceLayer & layer,
    unsigned int & size_x, unsigned int & size_y,
    double & origin_x, double & origin_y, double & resolution)
  {
    size_x = layer.fence_size_x_;
    size_y = layer.fence_size_y_;
    origin_x = layer.fence_origin_x_;
    origin_y = layer.fence_origin_y_;
    resolution = layer.fence_resolution_;
  }
};

class FenceLayerTest : public ::testing::Test
{
public:
  FenceLayerTest()
  : layers_("map", false, false)
  {
  }

  void SetUp() override
  {
    node_ = std::make_shared<TestLifecycleNode>("fence_layer_test_node");
    tf_ = std::make_shared<nav2::TransformBuffer>(node_->get_clock());
    tf_->setUsingDedicatedThread(true);

    fence_layer_ = std::make_shared<FenceLayer>();
    fence_layer_->initialize(
      &layers_, "fence_layer", tf_.get(), node_, nullptr);
    layers_.addPlugin(std::shared_ptr<Layer>(fence_layer_));

    // Initialize layered costmap with default size (20x20 cells at 0.1m, origin (0, 0))
    layers_.resizeMap(20, 20, 0.1, 0.0, 0.0);
  }

  void TearDown() override
  {
    fence_layer_->deactivate();
    fence_layer_.reset();
    tf_.reset();
    node_.reset();
  }

  geometry_msgs::msg::PolygonStamped createSquarePolygon(
    double min_x, double min_y, double max_x, double max_y,
    const std::string & frame_id = "map")
  {
    geometry_msgs::msg::PolygonStamped poly;
    poly.header.frame_id = frame_id;
    poly.header.stamp = node_->now();

    geometry_msgs::msg::Point32 p1, p2, p3, p4;
    p1.x = min_x; p1.y = min_y;
    p2.x = max_x; p2.y = min_y;
    p3.x = max_x; p3.y = max_y;
    p4.x = min_x; p4.y = max_y;

    poly.polygon.points = {p1, p2, p3, p4};
    return poly;
  }

  void updateBoundsAndCosts()
  {
    layers_.updateMap(0.0, 0.0, 0.0);
  }

protected:
  std::shared_ptr<TestLifecycleNode> node_;
  std::shared_ptr<nav2::TransformBuffer> tf_;
  LayeredCostmap layers_;
  std::shared_ptr<FenceLayer> fence_layer_;
};

// 1. Ray casting on convex polygon
TEST_F(FenceLayerTest, testPointInPolygonConvex)
{
  FenceLayer layer;
  // Square from (0,0) to (10,10)
  std::vector<double> px = {0.0, 10.0, 10.0, 0.0};
  std::vector<double> py = {0.0, 0.0, 10.0, 10.0};
  FenceLayerTester::setPolygonDirect(layer, px, py);

  // Center
  EXPECT_TRUE(FenceLayerTester::isPointInPolygon(layer, 5.0, 5.0));
  // Near corners inside
  EXPECT_TRUE(FenceLayerTester::isPointInPolygon(layer, 1.0, 1.0));
  EXPECT_TRUE(FenceLayerTester::isPointInPolygon(layer, 9.0, 9.0));

  // Outside points
  EXPECT_FALSE(FenceLayerTester::isPointInPolygon(layer, -1.0, 5.0));
  EXPECT_FALSE(FenceLayerTester::isPointInPolygon(layer, 11.0, 5.0));
  EXPECT_FALSE(FenceLayerTester::isPointInPolygon(layer, 5.0, -1.0));
  EXPECT_FALSE(FenceLayerTester::isPointInPolygon(layer, 5.0, 11.0));
  EXPECT_FALSE(FenceLayerTester::isPointInPolygon(layer, 15.0, 15.0));
}

// 2. Ray casting on concave polygon (U-shape)
TEST_F(FenceLayerTest, testPointInPolygonConcave)
{
  FenceLayer layer;
  // U-shape polygon
  // (0,0) -> (10,0) -> (10,10) -> (7,10) -> (7,3) -> (3,3) -> (3,10) -> (0,10)
  std::vector<double> px = {0.0, 10.0, 10.0, 7.0, 7.0, 3.0, 3.0, 0.0};
  std::vector<double> py = {0.0, 0.0, 10.0, 10.0, 3.0, 3.0, 10.0, 10.0};
  FenceLayerTester::setPolygonDirect(layer, px, py);

  // Bottom bar (inside)
  EXPECT_TRUE(FenceLayerTester::isPointInPolygon(layer, 5.0, 1.5));
  // Left arm (inside)
  EXPECT_TRUE(FenceLayerTester::isPointInPolygon(layer, 1.5, 6.0));
  // Right arm (inside)
  EXPECT_TRUE(FenceLayerTester::isPointInPolygon(layer, 8.5, 6.0));

  // Cavity in middle of U (outside)
  EXPECT_FALSE(FenceLayerTester::isPointInPolygon(layer, 5.0, 6.0));
  EXPECT_FALSE(FenceLayerTester::isPointInPolygon(layer, 5.0, 9.0));

  // Far outside
  EXPECT_FALSE(FenceLayerTester::isPointInPolygon(layer, -5.0, 5.0));
  EXPECT_FALSE(FenceLayerTester::isPointInPolygon(layer, 15.0, 5.0));
}

// 3. Points on or near polygon edge
TEST_F(FenceLayerTest, testPointInPolygonOnEdge)
{
  FenceLayer layer;
  std::vector<double> px = {0.0, 10.0, 10.0, 0.0};
  std::vector<double> py = {0.0, 0.0, 10.0, 10.0};
  FenceLayerTester::setPolygonDirect(layer, px, py);

  // Slightly inside vs slightly outside
  EXPECT_TRUE(FenceLayerTester::isPointInPolygon(layer, 0.01, 5.0));
  EXPECT_FALSE(FenceLayerTester::isPointInPolygon(layer, -0.01, 5.0));
  EXPECT_TRUE(FenceLayerTester::isPointInPolygon(layer, 9.99, 5.0));
  EXPECT_FALSE(FenceLayerTester::isPointInPolygon(layer, 10.01, 5.0));
}

// 4. Costmap resizes to polygon bounding box + circumscribed radius padding
TEST_F(FenceLayerTest, testPolygonResize)
{
  auto poly_msg = std::make_shared<geometry_msgs::msg::PolygonStamped>(
    createSquarePolygon(2.0, 3.0, 8.0, 9.0));

  FenceLayerTester::triggerPolygonCallback(*fence_layer_, poly_msg);
  EXPECT_FALSE(fence_layer_->isCurrent());
  EXPECT_TRUE(FenceLayerTester::isPolygonUpdated(*fence_layer_));

  updateBoundsAndCosts();

  EXPECT_TRUE(fence_layer_->isCurrent());
  EXPECT_TRUE(FenceLayerTester::hasReceivedPolygon(*fence_layer_));

  Costmap2D * master = layers_.getCostmap();
  double circ_r = layers_.getCircumscribedRadius();
  double res = master->getResolution();
  double padding = std::ceil(circ_r / res) * res;

  EXPECT_NEAR(master->getOriginX(), 2.0 - padding, 1e-5);
  EXPECT_NEAR(master->getOriginY(), 3.0 - padding, 1e-5);

  unsigned int expected_cells_x = static_cast<unsigned int>(
    std::ceil((8.0 + padding - (2.0 - padding)) / res));
  unsigned int expected_cells_y = static_cast<unsigned int>(
    std::ceil((9.0 + padding - (3.0 - padding)) / res));

  EXPECT_EQ(master->getSizeInCellsX(), expected_cells_x);
  EXPECT_EQ(master->getSizeInCellsY(), expected_cells_y);
}

// 5. All cells outside polygon are LETHAL_OBSTACLE
TEST_F(FenceLayerTest, testLethalOutside)
{
  // Polygon from (1.0, 1.0) to (4.0, 4.0)
  auto poly_msg = std::make_shared<geometry_msgs::msg::PolygonStamped>(
    createSquarePolygon(1.0, 1.0, 4.0, 4.0));

  FenceLayerTester::triggerPolygonCallback(*fence_layer_, poly_msg);
  updateBoundsAndCosts();

  Costmap2D * master = layers_.getCostmap();
  for (unsigned int j = 0; j < master->getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < master->getSizeInCellsX(); ++i) {
      double wx, wy;
      master->mapToWorld(i, j, wx, wy);
      bool inside = (wx > 1.0) && (wx < 4.0) && (wy > 1.0) && (wy < 4.0);
      if (!inside) {
        EXPECT_EQ(master->getCost(i, j), LETHAL_OBSTACLE)
          << "Cell (" << i << ", " << j << ") at world (" << wx << ", " << wy <<
          ") should be LETHAL";
      }
    }
  }
}

// 6. Cells inside polygon retain their original cost (not overwritten)
TEST_F(FenceLayerTest, testInsidePreserved)
{
  // Set master costmap default to FREE_SPACE (or a specific value)
  Costmap2D * master = layers_.getCostmap();
  master->setDefaultValue(FREE_SPACE);

  auto poly_msg = std::make_shared<geometry_msgs::msg::PolygonStamped>(
    createSquarePolygon(1.0, 1.0, 4.0, 4.0));

  FenceLayerTester::triggerPolygonCallback(*fence_layer_, poly_msg);
  updateBoundsAndCosts();

  // For a cell strictly inside (e.g. wx=2.5, wy=2.5)
  unsigned int mx, my;
  ASSERT_TRUE(master->worldToMap(2.5, 2.5, mx, my));
  EXPECT_EQ(master->getCost(mx, my), FREE_SPACE);
}

// 7. New polygon arrives, costmap resizes and mask updates
TEST_F(FenceLayerTest, testPolygonUpdate)
{
  // First polygon: [1.0, 1.0] to [3.0, 3.0]
  auto poly_msg1 = std::make_shared<geometry_msgs::msg::PolygonStamped>(
    createSquarePolygon(1.0, 1.0, 3.0, 3.0));
  FenceLayerTester::triggerPolygonCallback(*fence_layer_, poly_msg1);
  updateBoundsAndCosts();

  Costmap2D * master = layers_.getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(master->worldToMap(2.0, 2.0, mx, my));
  EXPECT_NE(master->getCost(mx, my), LETHAL_OBSTACLE);

  // Second polygon shifted: [5.0, 5.0] to [10.0, 10.0]
  auto poly_msg2 = std::make_shared<geometry_msgs::msg::PolygonStamped>(
    createSquarePolygon(5.0, 5.0, 10.0, 10.0));
  FenceLayerTester::triggerPolygonCallback(*fence_layer_, poly_msg2);
  updateBoundsAndCosts();

  // (2.0, 2.0) is now outside the costmap or outside the new fence
  if (master->worldToMap(2.0, 2.0, mx, my)) {
    EXPECT_EQ(master->getCost(mx, my), LETHAL_OBSTACLE);
  }
  // (7.0, 7.0) is inside the new polygon
  ASSERT_TRUE(master->worldToMap(7.0, 7.0, mx, my));
  EXPECT_NE(master->getCost(mx, my), LETHAL_OBSTACLE);
}

// 8. Layer is a no-op before first polygon arrives
TEST_F(FenceLayerTest, testNoPolygon)
{
  EXPECT_TRUE(fence_layer_->isCurrent());
  EXPECT_FALSE(FenceLayerTester::hasReceivedPolygon(*fence_layer_));

  double min_x = 1e6, min_y = 1e6, max_x = -1e6, max_y = -1e6;
  fence_layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  // Bounds should not have been updated since no polygon received
  EXPECT_EQ(min_x, 1e6);
  EXPECT_EQ(max_x, -1e6);
}

// 9. Layer does nothing when enabled: false
TEST_F(FenceLayerTest, testDisabled)
{
  auto poly_msg = std::make_shared<geometry_msgs::msg::PolygonStamped>(
    createSquarePolygon(1.0, 1.0, 4.0, 4.0));

  // Disable layer via dynamic parameter
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("fence_layer.enabled", false)
  };
  fence_layer_->activate();
  auto res = node_->set_parameters(params);
  EXPECT_TRUE(res[0].successful);
  EXPECT_FALSE(fence_layer_->isEnabled());

  FenceLayerTester::triggerPolygonCallback(*fence_layer_, poly_msg);

  double min_x = 1e6, min_y = 1e6, max_x = -1e6, max_y = -1e6;
  fence_layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_EQ(min_x, 1e6);
}

// 10. Polygon in non-global frame is transformed correctly
TEST_F(FenceLayerTest, testTfTransform)
{
  // Broadcast transform from "odom" to "map": translation x=5.0, y=5.0
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = node_->now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "odom";
  tf_msg.transform.translation.x = 5.0;
  tf_msg.transform.translation.y = 5.0;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.w = 1.0;

  tf_->setTransform(tf_msg, "test_authority", false);

  // Polygon in "odom" frame: [0, 0] to [2, 2] -> transformed to "map": [5, 5] to [7, 7]
  auto poly_msg = std::make_shared<geometry_msgs::msg::PolygonStamped>(
    createSquarePolygon(0.0, 0.0, 2.0, 2.0, "odom"));

  FenceLayerTester::triggerPolygonCallback(*fence_layer_, poly_msg);
  updateBoundsAndCosts();

  Costmap2D * master = layers_.getCostmap();
  unsigned int mx, my;
  // (6.0, 6.0) in "map" should be inside
  ASSERT_TRUE(master->worldToMap(6.0, 6.0, mx, my));
  EXPECT_NE(master->getCost(mx, my), LETHAL_OBSTACLE);

  // (1.0, 1.0) in "map" should be outside
  if (master->worldToMap(1.0, 1.0, mx, my)) {
    EXPECT_EQ(master->getCost(mx, my), LETHAL_OBSTACLE);
  }
}

// 11. Grid extends beyond polygon by at least circumscribed radius
TEST_F(FenceLayerTest, testPaddingCircumscribedRadius)
{
  layers_.setFootprint(nav2_costmap_2d::makeFootprintFromRadius(0.55));
  auto poly_msg = std::make_shared<geometry_msgs::msg::PolygonStamped>(
    createSquarePolygon(0.0, 0.0, 10.0, 10.0));

  FenceLayerTester::triggerPolygonCallback(*fence_layer_, poly_msg);
  updateBoundsAndCosts();

  Costmap2D * master = layers_.getCostmap();
  double expected_padding = std::ceil(0.55 / master->getResolution()) * master->getResolution();

  EXPECT_LE(master->getOriginX(), 0.0 - expected_padding);
  EXPECT_LE(master->getOriginY(), 0.0 - expected_padding);
}

// 12. Polygon with 3 points works correctly
TEST_F(FenceLayerTest, testSmallPolygon)
{
  geometry_msgs::msg::PolygonStamped poly;
  poly.header.frame_id = "map";
  geometry_msgs::msg::Point32 p1, p2, p3;
  p1.x = 0.0; p1.y = 0.0;
  p2.x = 4.0; p2.y = 0.0;
  p3.x = 2.0; p3.y = 4.0;
  poly.polygon.points = {p1, p2, p3};

  auto poly_msg = std::make_shared<geometry_msgs::msg::PolygonStamped>(poly);
  FenceLayerTester::triggerPolygonCallback(*fence_layer_, poly_msg);
  updateBoundsAndCosts();

  EXPECT_TRUE(FenceLayerTester::hasReceivedPolygon(*fence_layer_));

  Costmap2D * master = layers_.getCostmap();
  unsigned int mx, my;
  // Centroid (2.0, 1.33) is inside triangle
  ASSERT_TRUE(master->worldToMap(2.0, 1.33, mx, my));
  EXPECT_NE(master->getCost(mx, my), LETHAL_OBSTACLE);

  // (0.5, 3.0) is outside triangle
  ASSERT_TRUE(master->worldToMap(0.5, 3.0, mx, my));
  EXPECT_EQ(master->getCost(mx, my), LETHAL_OBSTACLE);
}

// 13. Polygon with < 3 points is rejected
TEST_F(FenceLayerTest, testInvalidPolygon)
{
  geometry_msgs::msg::PolygonStamped poly;
  poly.header.frame_id = "map";
  geometry_msgs::msg::Point32 p1, p2;
  p1.x = 0.0; p1.y = 0.0;
  p2.x = 4.0; p2.y = 0.0;
  poly.polygon.points = {p1, p2};

  auto poly_msg = std::make_shared<geometry_msgs::msg::PolygonStamped>(poly);
  FenceLayerTester::triggerPolygonCallback(*fence_layer_, poly_msg);

  EXPECT_FALSE(FenceLayerTester::hasReceivedPolygon(*fence_layer_));
  EXPECT_FALSE(FenceLayerTester::isPolygonUpdated(*fence_layer_));
}

// 14. Reset layer
TEST_F(FenceLayerTest, testReset)
{
  auto poly_msg = std::make_shared<geometry_msgs::msg::PolygonStamped>(
    createSquarePolygon(1.0, 1.0, 4.0, 4.0));

  FenceLayerTester::triggerPolygonCallback(*fence_layer_, poly_msg);
  updateBoundsAndCosts();
  EXPECT_TRUE(FenceLayerTester::hasReceivedPolygon(*fence_layer_));

  fence_layer_->reset();
  EXPECT_FALSE(FenceLayerTester::hasReceivedPolygon(*fence_layer_));
  EXPECT_TRUE(fence_layer_->isCurrent());
}

// 15. isClearable returns false
TEST_F(FenceLayerTest, testIsClearable)
{
  EXPECT_FALSE(fence_layer_->isClearable());
}

}  // namespace nav2_costmap_2d

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
