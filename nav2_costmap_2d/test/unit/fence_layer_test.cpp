// Copyright (c) 2026, Aniruddh Yelluri
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
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/fence_layer.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "nav2_msgs/srv/set_fence.hpp"


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

/**
 * @brief Testable subclass that exposes the protected setFenceCallback
 */
class TestableFenceLayer : public nav2_costmap_2d::FenceLayer
{
public:
  void callSetFence(
    const std::shared_ptr<nav2_msgs::srv::SetFence::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetFence::Response> response)
  {
    auto header = std::make_shared<rmw_request_id_t>();
    setFenceCallback(header, request, response);
  }
};

/**
 * @brief Helper to add a TestableFenceLayer to a LayeredCostmap
 */
void addFenceLayer(
  nav2_costmap_2d::LayeredCostmap & layers,
  nav2::TransformBuffer & tf, nav2::LifecycleNode::SharedPtr node,
  std::shared_ptr<TestableFenceLayer> & flayer,
  const std::string & name = "fence",
  rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
{
  flayer = std::make_shared<TestableFenceLayer>();
  flayer->initialize(&layers, name, &tf, node, callback_group);
  layers.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(flayer));
}

/**
 * @brief Build a SetFence request from a list of (x,y) pairs
 */
std::shared_ptr<nav2_msgs::srv::SetFence::Request> makeFenceRequest(
  const std::string & frame_id,
  const std::vector<std::pair<float, float>> & points)
{
  auto request = std::make_shared<nav2_msgs::srv::SetFence::Request>();
  request->fence.header.frame_id = frame_id;
  for (const auto & [x, y] : points) {
    geometry_msgs::msg::Point32 p;
    p.x = x;
    p.y = y;
    p.z = 0.0f;
    request->fence.polygon.points.push_back(p);
  }
  return request;
}

class FenceLayerTest : public ::testing::Test
{
public:
  FenceLayerTest()
  : layers_("frame", false, false)
  {
    node_ = std::make_shared<TestLifecycleNode>("fence_layer_test_node");

    // 10x10 cells at 1.0 m/cell, origin at (0,0)
    // Cell centers are at (0.5, 0.5), (1.5, 1.5), ...
    layers_.resizeMap(10, 10, 1.0, 0.0, 0.0);
    nav2::TransformBuffer tf(node_->get_clock());
    addFenceLayer(layers_, tf, node_, fence_layer_);
  }

  ~FenceLayerTest() {}

  void setFence(const std::vector<std::pair<float, float>> & points)
  {
    auto request = makeFenceRequest("frame", points);
    auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
    fence_layer_->callSetFence(request, response);
    ASSERT_TRUE(response->success) << response->message;
  }

  void clearFence()
  {
    auto request = makeFenceRequest("frame", {});
    auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
    fence_layer_->callSetFence(request, response);
    ASSERT_TRUE(response->success) << response->message;
  }

  void updateLayer()
  {
    double min_x = 1e6, min_y = 1e6, max_x = -1e6, max_y = -1e6;
    fence_layer_->updateBounds(5.0, 5.0, 0.0, &min_x, &min_y, &max_x, &max_y);

    nav2_costmap_2d::Costmap2D * costmap = layers_.getCostmap();
    fence_layer_->updateCosts(
      *costmap, 0, 0,
      static_cast<int>(costmap->getSizeInCellsX()),
      static_cast<int>(costmap->getSizeInCellsY()));
  }

protected:
  std::shared_ptr<TestableFenceLayer> fence_layer_;
  std::shared_ptr<TestLifecycleNode> node_;
  nav2_costmap_2d::LayeredCostmap layers_;
};

/**
 * Test: No fence set — layer should not modify the costmap.
 */
TEST_F(FenceLayerTest, NoFenceSet)
{
  nav2_costmap_2d::Costmap2D * costmap = layers_.getCostmap();
  costmap->resetMap(0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

  updateLayer();

  for (unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
      EXPECT_EQ(costmap->getCost(i, j), nav2_costmap_2d::FREE_SPACE)
        << "Cell (" << i << ", " << j << ") should be FREE_SPACE";
    }
  }
}

/**
 * Test: Rectangular fence — cells inside should remain free,
 * cells outside should be LETHAL_OBSTACLE.
 */
TEST_F(FenceLayerTest, RectangularFence)
{
  nav2_costmap_2d::Costmap2D * costmap = layers_.getCostmap();
  costmap->resetMap(0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

  // Rectangle from (2.0, 2.0) to (7.0, 7.0)
  setFence({{2.0f, 2.0f}, {7.0f, 2.0f}, {7.0f, 7.0f}, {2.0f, 7.0f}});
  updateLayer();

  for (unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
      double wx, wy;
      costmap->mapToWorld(i, j, wx, wy);
      unsigned char cost = costmap->getCost(i, j);

      if (wx > 2.0 && wx < 7.0 && wy > 2.0 && wy < 7.0) {
        EXPECT_EQ(cost, nav2_costmap_2d::FREE_SPACE)
          << "Cell (" << i << ", " << j << ") at world (" << wx << ", " << wy
          << ") should be FREE_SPACE (inside fence)";
      } else if (wx < 2.0 || wx > 7.0 || wy < 2.0 || wy > 7.0) {
        EXPECT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE)
          << "Cell (" << i << ", " << j << ") at world (" << wx << ", " << wy
          << ") should be LETHAL_OBSTACLE (outside fence)";
      }
    }
  }
}

/**
 * Test: Clear fence via empty polygon — all cells should revert to FREE_SPACE.
 */
TEST_F(FenceLayerTest, ClearFence)
{
  nav2_costmap_2d::Costmap2D * costmap = layers_.getCostmap();
  costmap->resetMap(0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

  // Set then clear a fence
  setFence({{3.0f, 3.0f}, {6.0f, 3.0f}, {6.0f, 6.0f}, {3.0f, 6.0f}});
  clearFence();

  // Reset costmap after fence is cleared
  costmap->resetMap(0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  updateLayer();

  for (unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
      EXPECT_EQ(costmap->getCost(i, j), nav2_costmap_2d::FREE_SPACE)
        << "Cell (" << i << ", " << j << ") should be FREE_SPACE after fence cleared";
    }
  }
}

/**
 * Test: Fence polygon with < 3 vertices is rejected.
 */
TEST_F(FenceLayerTest, RejectTooFewVertices)
{
  auto request = makeFenceRequest("frame", {{1.0f, 1.0f}, {2.0f, 2.0f}});
  auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  fence_layer_->callSetFence(request, response);

  EXPECT_FALSE(response->success);
}

/**
 * Test: Layer is not clearable.
 */
TEST_F(FenceLayerTest, NotClearable)
{
  EXPECT_FALSE(fence_layer_->isClearable());
}

/**
 * Test: Fence from parameter at initialization.
 */
TEST(FenceLayerParameterTest, FenceFromParameter)
{
  auto node = std::make_shared<TestLifecycleNode>("fence_param_test_node");

  // Declare the fence_polygon parameter BEFORE initializing the layer
  // Rectangle from (1.0, 1.0) to (8.0, 8.0)
  std::vector<double> polygon_param = {1.0, 1.0, 8.0, 1.0, 8.0, 8.0, 1.0, 8.0};
  node->declare_parameter(
    "fence_param.fence_polygon",
    rclcpp::ParameterValue(polygon_param));

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1.0, 0.0, 0.0);
  nav2::TransformBuffer tf(node->get_clock());

  auto fence_layer = std::make_shared<TestableFenceLayer>();
  fence_layer->initialize(&layers, "fence_param", &tf, node, nullptr);
  layers.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(fence_layer));

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  costmap->resetMap(0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

  double min_x = 1e6, min_y = 1e6, max_x = -1e6, max_y = -1e6;
  fence_layer->updateBounds(5.0, 5.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  fence_layer->updateCosts(
    *costmap, 0, 0,
    static_cast<int>(costmap->getSizeInCellsX()),
    static_cast<int>(costmap->getSizeInCellsY()));

  // Cell (0,0) center is at world (0.5, 0.5) — outside [1,8]x[1,8] → lethal
  EXPECT_EQ(costmap->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);

  // Cell (4,4) center is at world (4.5, 4.5) — inside [1,8]x[1,8] → free
  EXPECT_EQ(costmap->getCost(4, 4), nav2_costmap_2d::FREE_SPACE);
}

/**
 * Test: Update fence at runtime — new polygon replaces the old one.
 */
TEST_F(FenceLayerTest, UpdateFenceAtRuntime)
{
  nav2_costmap_2d::Costmap2D * costmap = layers_.getCostmap();

  // First fence: small square (3,3)-(6,6)
  costmap->resetMap(0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  setFence({{3.0f, 3.0f}, {6.0f, 3.0f}, {6.0f, 6.0f}, {3.0f, 6.0f}});
  updateLayer();

  // Cell (0,0) at (0.5, 0.5) should be lethal (outside small fence)
  EXPECT_EQ(costmap->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  // Cell (4,4) at (4.5, 4.5) should be free (inside small fence)
  EXPECT_EQ(costmap->getCost(4, 4), nav2_costmap_2d::FREE_SPACE);

  // Second fence: larger square (0,0)-(10,10) covering everything
  costmap->resetMap(0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  setFence({{-0.1f, -0.1f}, {10.1f, -0.1f}, {10.1f, 10.1f}, {-0.1f, 10.1f}});
  updateLayer();

  // Now all cells should be free (everything is inside the big fence)
  for (unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
      EXPECT_EQ(costmap->getCost(i, j), nav2_costmap_2d::FREE_SPACE)
        << "Cell (" << i << ", " << j << ") should be FREE after large fence set";
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
