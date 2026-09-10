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
#include "nav2_costmap_2d/geofence_layer.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "nav2_msgs/srv/set_fence.hpp"


// Test Lifecycle Node

class TestLifecycleNode : public nav2::LifecycleNode
{
public:
  explicit TestLifecycleNode(const std::string & name)
  : nav2::LifecycleNode(name) {}

  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn onError(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
};

// Testable subclass that exposes the protected service callback

class TestableGeofenceLayer : public nav2_costmap_2d::GeofenceLayer
{
public:
  void callSetFence(
    const std::shared_ptr<nav2_msgs::srv::SetFence::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetFence::Response> response)
  {
    auto header = std::make_shared<rmw_request_id_t>();
    setFenceCallback(header, request, response);
  }

  rcl_interfaces::msg::SetParametersResult callValidate(
    const std::vector<rclcpp::Parameter> & params)
  {
    return validateParameterUpdatesCallback(params);
  }

  bool hasFence() const {return has_fence_;}
  size_t maskSize() const {return outside_fence_.size();}
  bool outsideFenceAt(unsigned int i, unsigned int j) const
  {
    if (mask_size_x_ == 0) {return false;}
    return outside_fence_[j * mask_size_x_ + i];
  }
};

// Helpers

std::shared_ptr<nav2_msgs::srv::SetFence::Request> makeFenceRequest(
  const std::string & frame_id,
  const std::vector<std::pair<float, float>> & points)
{
  auto request = std::make_shared<nav2_msgs::srv::SetFence::Request>();
  request->fence.header.frame_id = frame_id;
  for (const auto & [x, y] : points) {
    geometry_msgs::msg::Point32 p;
    p.x = x; p.y = y; p.z = 0.0f;
    request->fence.polygon.points.push_back(p);
  }
  return request;
}

// Test Fixture

class GeofenceLayerTest : public ::testing::Test
{
public:
  GeofenceLayerTest()
  : layers_("map", false, false)
  {
    node_ = std::make_shared<TestLifecycleNode>("geofence_test_node");

    // 10x10 cells at 1.0 m/cell, origin at (0,0)
    layers_.resizeMap(10, 10, 1.0, 0.0, 0.0);

    tf_ = nav2::create_transform_buffer(node_);
    geofence_ = std::make_shared<TestableGeofenceLayer>();
    // Disable resize_to_fence for all fixture tests (tests a fixed-size map)
    node_->declare_parameter("geofence.resize_to_fence", false);
    geofence_->initialize(&layers_, "geofence", tf_.get(), node_, nullptr);
    layers_.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(geofence_));
  }

  void setFence(const std::vector<std::pair<float, float>> & points, bool expect_success = true)
  {
    auto request = makeFenceRequest("map", points);
    auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
    geofence_->callSetFence(request, response);
    if (expect_success) {
      ASSERT_TRUE(response->success) << response->message;
    }
  }

  void clearFence()
  {
    auto request = makeFenceRequest("map", {});
    auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
    geofence_->callSetFence(request, response);
    ASSERT_TRUE(response->success);
  }

  void updateLayer()
  {
    double min_x = 1e6, min_y = 1e6, max_x = -1e6, max_y = -1e6;
    geofence_->updateBounds(5.0, 5.0, 0.0, &min_x, &min_y, &max_x, &max_y);
    nav2_costmap_2d::Costmap2D * costmap = layers_.getCostmap();
    geofence_->updateCosts(
      *costmap, 0, 0,
      static_cast<int>(costmap->getSizeInCellsX()),
      static_cast<int>(costmap->getSizeInCellsY()));
  }

  nav2_costmap_2d::Costmap2D * resetAndGetCostmap()
  {
    nav2_costmap_2d::Costmap2D * c = layers_.getCostmap();
    c->resetMap(0, 0, c->getSizeInCellsX(), c->getSizeInCellsY());
    return c;
  }

protected:
  std::shared_ptr<TestableGeofenceLayer> geofence_;
  std::shared_ptr<TestLifecycleNode> node_;
  nav2::TransformBuffer::SharedPtr tf_;
  nav2_costmap_2d::LayeredCostmap layers_;
};

// Tests

// 1. No fence set — layer should not modify the costmap
TEST_F(GeofenceLayerTest, NoFenceSet)
{
  auto * costmap = resetAndGetCostmap();
  updateLayer();
  for (unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
      EXPECT_EQ(costmap->getCost(i, j), nav2_costmap_2d::FREE_SPACE)
        << "Cell (" << i << "," << j << ") should be FREE_SPACE with no fence";
    }
  }
}

// 2. Rectangular fence — cells inside free, cells outside lethal
TEST_F(GeofenceLayerTest, RectangularFence)
{
  auto * costmap = resetAndGetCostmap();
  setFence({{2.0f, 2.0f}, {7.0f, 2.0f}, {7.0f, 7.0f}, {2.0f, 7.0f}});
  updateLayer();

  for (unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
      double wx, wy;
      costmap->mapToWorld(i, j, wx, wy);
      unsigned char cost = costmap->getCost(i, j);
      if (wx > 2.0 && wx < 7.0 && wy > 2.0 && wy < 7.0) {
        EXPECT_EQ(cost, nav2_costmap_2d::FREE_SPACE)
          << "Cell (" << i << "," << j << ") at (" << wx << "," << wy << ") should be free";
      } else if (wx < 2.0 || wx > 7.0 || wy < 2.0 || wy > 7.0) {
        EXPECT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE)
          << "Cell (" << i << "," << j << ") at (" << wx << "," << wy << ") should be lethal";
      }
    }
  }
}

// 3. Clear fence — all cells should revert to FREE_SPACE
TEST_F(GeofenceLayerTest, ClearFence)
{
  setFence({{3.0f, 3.0f}, {6.0f, 3.0f}, {6.0f, 6.0f}, {3.0f, 6.0f}});
  clearFence();

  auto * costmap = resetAndGetCostmap();
  updateLayer();

  for (unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
      EXPECT_EQ(costmap->getCost(i, j), nav2_costmap_2d::FREE_SPACE)
        << "Cell (" << i << "," << j << ") should be FREE_SPACE after fence cleared";
    }
  }
}

// 4. Reject polygon with < 3 vertices
TEST_F(GeofenceLayerTest, RejectTooFewVertices)
{
  auto request = makeFenceRequest("map", {{1.0f, 1.0f}, {2.0f, 2.0f}});
  auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  geofence_->callSetFence(request, response);
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(geofence_->hasFence());
}

// 5. GeofenceLayer is not clearable
TEST_F(GeofenceLayerTest, NotClearable)
{
  EXPECT_FALSE(geofence_->isClearable());
}

// 6. Update fence at runtime — new polygon replaces the old one
TEST_F(GeofenceLayerTest, UpdateFenceAtRuntime)
{
  auto * costmap = resetAndGetCostmap();
  // Small fence: cells at (0,0) = world (0.5,0.5) should be lethal, (4,4) free
  setFence({{3.0f, 3.0f}, {6.0f, 3.0f}, {6.0f, 6.0f}, {3.0f, 6.0f}});
  updateLayer();
  EXPECT_EQ(costmap->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(costmap->getCost(4, 4), nav2_costmap_2d::FREE_SPACE);

  // Large fence covering everything — now all cells should be free
  costmap = resetAndGetCostmap();
  setFence({{-0.1f, -0.1f}, {10.1f, -0.1f}, {10.1f, 10.1f}, {-0.1f, 10.1f}});
  updateLayer();
  for (unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
      EXPECT_EQ(costmap->getCost(i, j), nav2_costmap_2d::FREE_SPACE)
        << "Cell (" << i << "," << j << ") should be FREE after large fence";
    }
  }
}

// 7. reset() clears all state
TEST_F(GeofenceLayerTest, ResetClearsState)
{
  setFence({{1.0f, 1.0f}, {8.0f, 1.0f}, {8.0f, 8.0f}, {1.0f, 8.0f}});
  updateLayer();
  EXPECT_TRUE(geofence_->hasFence());
  EXPECT_GT(geofence_->maskSize(), 0u);

  geofence_->reset();
  EXPECT_FALSE(geofence_->hasFence());
  EXPECT_EQ(geofence_->maskSize(), 0u);
  EXPECT_TRUE(geofence_->isCurrent());
}

// 8. Parameter validation rejects bad fence_polygon string
TEST_F(GeofenceLayerTest, ValidateParameterRejectsBadString)
{
  // Call the validator directly — no need for activate() or a running node
  auto result = geofence_->callValidate(
    {rclcpp::Parameter("geofence.fence_polygon", std::string("not_a_valid_polygon"))});
  EXPECT_FALSE(result.successful);
}

// 9. Parameter validation rejects wrong type for 'enabled'
TEST_F(GeofenceLayerTest, ValidateParameterRejectsBadEnabledType)
{
  // Call the validator directly — enabled must be a bool, not a string
  auto result = geofence_->callValidate(
    {rclcpp::Parameter("geofence.enabled", std::string("yes"))});
  EXPECT_FALSE(result.successful);
}

// Standalone test: load fence from parameter at init

TEST(GeofenceLayerInitTest, FenceFromParameter)
{
  auto node = std::make_shared<TestLifecycleNode>("geofence_init_test_node");

  // Declare params BEFORE initialize()
  node->declare_parameter("gf.fence_polygon", "[[1.0, 1.0], [8.0, 1.0], [8.0, 8.0], [1.0, 8.0]]");
  node->declare_parameter("gf.resize_to_fence", false);

  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto tf = nav2::create_transform_buffer(node);

  auto layer = std::make_shared<TestableGeofenceLayer>();
  layer->initialize(&layers, "gf", tf.get(), node, nullptr);
  layers.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(layer));

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  costmap->resetMap(0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

  double min_x = 1e6, min_y = 1e6, max_x = -1e6, max_y = -1e6;
  layer->updateBounds(5.0, 5.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  layer->updateCosts(
    *costmap, 0, 0,
    static_cast<int>(costmap->getSizeInCellsX()),
    static_cast<int>(costmap->getSizeInCellsY()));

  // Cell (0,0) center at world (0.5,0.5) — outside [1,8]x[1,8] → LETHAL
  EXPECT_EQ(costmap->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  // Cell (4,4) center at world (4.5,4.5) — inside [1,8]x[1,8] → FREE
  EXPECT_EQ(costmap->getCost(4, 4), nav2_costmap_2d::FREE_SPACE);
}

// Standalone test: bad fence_polygon at init throws

TEST(GeofenceLayerInitTest, BadFencePolygonThrows)
{
  auto node = std::make_shared<TestLifecycleNode>("geofence_throw_test_node");
  node->declare_parameter("bad.fence_polygon", "not_a_polygon");
  node->declare_parameter("bad.resize_to_fence", false);

  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto tf = nav2::create_transform_buffer(node);

  auto layer = std::make_shared<TestableGeofenceLayer>();
  EXPECT_THROW(
    layer->initialize(&layers, "bad", tf.get(), node, nullptr),
    std::runtime_error);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
