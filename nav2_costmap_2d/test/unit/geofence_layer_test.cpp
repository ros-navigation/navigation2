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

  void callUpdate(
    const std::vector<rclcpp::Parameter> & params)
  {
    updateParametersCallback(params);
  }

  bool hasFence() const {return has_fence_;}
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

// 2. Rectangular fence — interior cells free, border cells lethal, far-outside free
TEST_F(GeofenceLayerTest, RectangularFence)
{
  auto * costmap = resetAndGetCostmap();
  // Fence: world coords [[2,2],[7,2],[7,7],[2,7]], costmap 10x10 at 1m/cell
  setFence({{2.0f, 2.0f}, {7.0f, 2.0f}, {7.0f, 7.0f}, {2.0f, 7.0f}});
  updateLayer();

  // Cell (4,4) at world (4.5,4.5) — well inside polygon → FREE
  EXPECT_EQ(costmap->getCost(4, 4), nav2_costmap_2d::FREE_SPACE)
    << "Interior cell should be FREE";
  EXPECT_EQ(costmap->getCost(4, 3), nav2_costmap_2d::FREE_SPACE)
    << "Interior cell should be FREE";

  // Cell (1,4) at world (1.5,4.5) — 0.5m outside the x=2 edge, within border_thickness=3 → LETHAL
  EXPECT_EQ(costmap->getCost(1, 4), nav2_costmap_2d::LETHAL_OBSTACLE)
    << "Cell just outside fence edge should be LETHAL";
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

// 7. reset() flags for re-write but preserves fence state
TEST_F(GeofenceLayerTest, ResetReappliesFence)
{
  setFence({{3.0f, 3.0f}, {6.0f, 3.0f}, {6.0f, 6.0f}, {3.0f, 6.0f}});
  updateLayer();
  EXPECT_TRUE(geofence_->hasFence());

  geofence_->reset();

  // Fence is preserved — reset only flags for re-write
  EXPECT_TRUE(geofence_->hasFence());
  EXPECT_FALSE(geofence_->isCurrent());

  // After the next update cycle the fence is re-applied
  auto * costmap = resetAndGetCostmap();
  updateLayer();
  EXPECT_EQ(costmap->getCost(4, 4), nav2_costmap_2d::FREE_SPACE)
    << "Interior cell should be FREE after re-rasterize";
  // Cell outside the fence should be LETHAL after re-rasterize (border expands correctly)
  EXPECT_EQ(costmap->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE)
    << "Outside-fence cell should be LETHAL after reset re-rasterize";
}

// 8. Parameter validation rejects bad fence_polygon string
TEST_F(GeofenceLayerTest, ValidateParameterRejectsBadString)
{
  // Call the validator directly — no need for activate() or a running node
  auto result = geofence_->callValidate(
    {rclcpp::Parameter("geofence.fence_polygon", std::string("not_a_valid_polygon"))});
  EXPECT_FALSE(result.successful);
}

// 9. Parameter validation rejects wrong types and updateParametersCallback coverage
TEST_F(GeofenceLayerTest, ValidateParameterRejectsBadTypes)
{
  auto result = geofence_->callValidate({rclcpp::Parameter("geofence.enabled",
      std::string("yes"))});
  EXPECT_FALSE(result.successful);

  result = geofence_->callValidate({rclcpp::Parameter("geofence.resize_to_fence",
      std::string("yes"))});
  EXPECT_FALSE(result.successful);

  result = geofence_->callValidate({rclcpp::Parameter("geofence.fence_polygon", 123)});
  EXPECT_FALSE(result.successful);

  result = geofence_->callValidate({rclcpp::Parameter("geofence.border_thickness", 1.5)});
  EXPECT_FALSE(result.successful);

  result = geofence_->callValidate({rclcpp::Parameter("geofence.border_thickness", -5)});
  EXPECT_FALSE(result.successful);
}

TEST_F(GeofenceLayerTest, UpdateParametersCallbackCoverage)
{
  // Cover the `continue` branch for unrelated params
  geofence_->callUpdate({rclcpp::Parameter("other_layer.enabled", false)});

  // Cover `enabled` branch
  geofence_->callUpdate({rclcpp::Parameter("geofence.enabled", false)});
  EXPECT_FALSE(geofence_->isEnabled());
  geofence_->callUpdate({rclcpp::Parameter("geofence.enabled", true)});

  // Cover `resize_to_fence` branch (normal map)
  geofence_->callUpdate({rclcpp::Parameter("geofence.resize_to_fence", true)});

  // Cover `fence_polygon` empty / clear
  geofence_->callUpdate({rclcpp::Parameter("geofence.fence_polygon", std::string("[]"))});

  // Cover `fence_polygon` update
  geofence_->callUpdate({rclcpp::Parameter("geofence.fence_polygon",
      std::string("[[1,1],[2,2],[3,3]]"))});
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

  // Cell (0,0) center at world (0.5,0.5) — 0.5m outside the x=1 edge,
  // within border_thickness=3 cells → LETHAL
  EXPECT_EQ(costmap->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  // Cell (4,4) center at world (4.5,4.5) — inside [1,8]x[1,8] → FREE
  EXPECT_EQ(costmap->getCost(4, 4), nav2_costmap_2d::FREE_SPACE);
  // Additional interior cells to confirm border-wall (not brute-force) semantics
  EXPECT_EQ(costmap->getCost(5, 5), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(costmap->getCost(3, 3), nav2_costmap_2d::FREE_SPACE);
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

// Standalone test: border_thickness=0 marks only outline cells LETHAL

TEST(GeofenceLayerBorderTest, BorderThicknessZero)
{
  auto node = std::make_shared<TestLifecycleNode>("geofence_border0_node");
  node->declare_parameter("gf0.fence_polygon", "");
  node->declare_parameter("gf0.resize_to_fence", false);
  node->declare_parameter("gf0.border_thickness", 0);

  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto tf = nav2::create_transform_buffer(node);

  auto layer = std::make_shared<TestableGeofenceLayer>();
  layer->initialize(&layers, "gf0", tf.get(), node, nullptr);
  layers.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(layer));

  // Set a fence via service
  auto request = std::make_shared<nav2_msgs::srv::SetFence::Request>();
  request->fence.header.frame_id = "map";
  for (const auto & [x, y] : std::vector<std::pair<float, float>>{
    {2.0f, 2.0f}, {7.0f, 2.0f}, {7.0f, 7.0f}, {2.0f, 7.0f}})
  {
    geometry_msgs::msg::Point32 p;
    p.x = x; p.y = y; p.z = 0.0f;
    request->fence.polygon.points.push_back(p);
  }
  auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  layer->callSetFence(request, response);
  ASSERT_TRUE(response->success);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  costmap->resetMap(0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

  double min_x = 1e6, min_y = 1e6, max_x = -1e6, max_y = -1e6;
  layer->updateBounds(5.0, 5.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  layer->updateCosts(
    *costmap, 0, 0,
    static_cast<int>(costmap->getSizeInCellsX()),
    static_cast<int>(costmap->getSizeInCellsY()));

  // Cell (4,4) — well inside polygon → FREE
  EXPECT_EQ(costmap->getCost(4, 4), nav2_costmap_2d::FREE_SPACE)
    << "Interior cell should be FREE with border_thickness=0";
  // Cell (0,0) — far outside, > 0 cells from outline → FREE with thickness=0
  EXPECT_EQ(costmap->getCost(0, 0), nav2_costmap_2d::FREE_SPACE)
    << "Far-outside cell should be FREE with border_thickness=0";
  // Cell (2,2) — on the outline → LETHAL even with border_thickness=0
  EXPECT_EQ(costmap->getCost(2, 2), nav2_costmap_2d::LETHAL_OBSTACLE)
    << "Outline cell should be LETHAL even with border_thickness=0";
}

// Regression: updateWithMax does not overwrite existing obstacles inside the fence

TEST_F(GeofenceLayerTest, DoesNotOverwriteObstacles)
{
  auto * costmap = resetAndGetCostmap();
  // Place an obstacle inside the fence area before setting the fence
  costmap->setCost(4, 4, nav2_costmap_2d::LETHAL_OBSTACLE);

  setFence({{3.0f, 3.0f}, {6.0f, 3.0f}, {6.0f, 6.0f}, {3.0f, 6.0f}});
  updateLayer();

  // Inside the fence our layer wrote FREE (0). updateWithMax keeps the higher value.
  EXPECT_EQ(costmap->getCost(4, 4), nav2_costmap_2d::LETHAL_OBSTACLE)
    << "Existing obstacle inside fence should not be cleared";
}

// Regression: no fence set — existing master costs are not modified

TEST_F(GeofenceLayerTest, NoFencePreservesExistingCosts)
{
  auto * costmap = resetAndGetCostmap();
  costmap->setCost(5, 5, nav2_costmap_2d::LETHAL_OBSTACLE);
  updateLayer();  // no fence set
  EXPECT_EQ(costmap->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE)
    << "No-fence update must not clear existing master costs";
}

// Regression: polygon extending beyond costmap bounds does not crash

TEST_F(GeofenceLayerTest, PartiallyOutOfBoundsPolygon)
{
  auto * costmap = resetAndGetCostmap();
  // Polygon extends well beyond the 10x10 costmap
  setFence({{-5.0f, -5.0f}, {15.0f, -5.0f}, {15.0f, 15.0f}, {-5.0f, 15.0f}});
  updateLayer();
  // All map cells are inside this oversized polygon → all FREE
  for (unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
      EXPECT_EQ(costmap->getCost(i, j), nav2_costmap_2d::FREE_SPACE)
        << "Cell (" << i << "," << j << ") should be FREE inside oversized fence";
    }
  }
}

// Regression: matchSize + reset causes re-rasterize on next update cycle

TEST_F(GeofenceLayerTest, MatchSizePreservesFence)
{
  setFence({{3.0f, 3.0f}, {6.0f, 3.0f}, {6.0f, 6.0f}, {3.0f, 6.0f}});
  updateLayer();
  EXPECT_TRUE(geofence_->hasFence());

  // Simulate what layered_costmap does after a resize: matchSize then reset
  layers_.resizeMap(12, 12, 1.0, 0.0, 0.0);
  geofence_->matchSize();
  geofence_->reset();

  nav2_costmap_2d::Costmap2D * costmap = layers_.getCostmap();
  costmap->resetMap(0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  updateLayer();

  // Fence polygon is preserved — interior cell should still be FREE
  EXPECT_EQ(costmap->getCost(4, 4), nav2_costmap_2d::FREE_SPACE)
    << "Interior cell should be FREE after matchSize+reset re-rasterize";
  EXPECT_TRUE(geofence_->hasFence());
}

// 10. updateBounds optimization — min/max only expanded when fence changes
TEST_F(GeofenceLayerTest, BoundsOptimizationTest)
{
  setFence({{3.0f, 3.0f}, {6.0f, 3.0f}, {6.0f, 6.0f}, {3.0f, 6.0f}});

  // First update: has_updated_data_ is true, so bounds should expand to the full map
  double min_x = 1e6, min_y = 1e6, max_x = -1e6, max_y = -1e6;
  geofence_->updateBounds(5.0, 5.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_DOUBLE_EQ(min_x, 0.0);
  EXPECT_DOUBLE_EQ(min_y, 0.0);
  EXPECT_DOUBLE_EQ(max_x, 10.0);
  EXPECT_DOUBLE_EQ(max_y, 10.0);

  // Second update: has_updated_data_ is false, bounds should not be touched
  min_x = 1e6; min_y = 1e6; max_x = -1e6; max_y = -1e6;
  geofence_->updateBounds(5.0, 5.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_DOUBLE_EQ(min_x, 1e6);
  EXPECT_DOUBLE_EQ(min_y, 1e6);
  EXPECT_DOUBLE_EQ(max_x, -1e6);
  EXPECT_DOUBLE_EQ(max_y, -1e6);
}

// 11. Regression: map completely outside polygon becomes completely LETHAL
TEST_F(GeofenceLayerTest, EntirelyOutOfBoundsPolygon)
{
  auto * costmap = resetAndGetCostmap();
  // Tiny polygon far away from the 10x10 map at origin
  setFence({{15.0f, 15.0f}, {16.0f, 15.0f}, {16.0f, 16.0f}, {15.0f, 16.0f}});
  updateLayer();

  // The map is entirely outside the geofence, so all cells must be LETHAL
  for (unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
      EXPECT_EQ(costmap->getCost(i, j), nav2_costmap_2d::LETHAL_OBSTACLE)
        << "Cell (" << i << "," << j << ") should be LETHAL outside entirely distant fence";
    }
  }
}

// 12. Regression: Parameter validation allows empty string to clear fence
TEST_F(GeofenceLayerTest, ValidateParameterClearsFence)
{
  auto result = geofence_->callValidate(
    {rclcpp::Parameter("geofence.fence_polygon", std::string(""))});
  EXPECT_TRUE(result.successful) << "Validator rejected empty string for clearance";

  result = geofence_->callValidate(
    {rclcpp::Parameter("geofence.fence_polygon", std::string("[]"))});
  EXPECT_TRUE(result.successful) << "Validator rejected [] for clearance";
}

// 13. Regression: Parameter validation rejects massive border_thickness
TEST_F(GeofenceLayerTest, ValidateParameterRejectsMassiveBorder)
{
  auto result = geofence_->callValidate(
    {rclcpp::Parameter("geofence.border_thickness", 1001)});
  EXPECT_FALSE(result.successful) << "Validator accepted massive border thickness";

  result = geofence_->callValidate(
    {rclcpp::Parameter("geofence.border_thickness", -1)});
  EXPECT_FALSE(result.successful) << "Validator accepted negative border thickness";
}

// 14. Dynamic Parameter Lifecycle: Updating parameter changes costmap
TEST_F(GeofenceLayerTest, DynamicParameterUpdateCostmap)
{
  geofence_->activate();

  // Init with small fence
  setFence({{2.0f, 2.0f}, {7.0f, 2.0f}, {7.0f, 7.0f}, {2.0f, 7.0f}});
  updateLayer();

  auto * costmap = layers_.getCostmap();

  // (0, 4) is 1.5m outside x=2. With default border_thickness=3, it should be LETHAL.
  EXPECT_EQ(costmap->getCost(0, 4), nav2_costmap_2d::LETHAL_OBSTACLE);

  // Dynamically update border_thickness to 0
  auto result = node_->set_parameter(rclcpp::Parameter("geofence.border_thickness", 0));
  ASSERT_EQ(result.successful, true) << "Failed to set border_thickness dynamically";

  // Process next update cycle
  costmap = resetAndGetCostmap();
  updateLayer();

  // Now, cell (0, 4) should be FREE_SPACE because border_thickness is 0
  EXPECT_EQ(costmap->getCost(0, 4), nav2_costmap_2d::FREE_SPACE);

  geofence_->deactivate();
}

// Regression: border BFS must expand outward only, not into polygon interior

TEST_F(GeofenceLayerTest, BorderExpandsOutwardOnly)
{
  auto * costmap = resetAndGetCostmap();
  // Fence {{2,2},{7,2},{7,7},{2,7}}: left boundary outline is at map x=2
  setFence({{2.0f, 2.0f}, {7.0f, 2.0f}, {7.0f, 7.0f}, {2.0f, 7.0f}});
  updateLayer();

  // Cell (1,4) is exterior, 1 cell left of the x=2 outline → LETHAL (within border_thickness=3)
  EXPECT_EQ(costmap->getCost(1, 4), nav2_costmap_2d::LETHAL_OBSTACLE)
    << "Cell just outside fence should be LETHAL";
  // Cell (3,4) is interior, 1 cell right of the x=2 outline → must stay FREE
  EXPECT_EQ(costmap->getCost(3, 4), nav2_costmap_2d::FREE_SPACE)
    << "Cell just inside fence should be FREE (border must not expand inward)";
}

// Regression: concave (L-shaped) polygon — notch is exterior, must be LETHAL within border
TEST_F(GeofenceLayerTest, ConcavePolygon)
{
  auto * costmap = resetAndGetCostmap();
  // L-shaped polygon: bottom arm x=[0,5] y=[0,3], left arm x=[0,3] y=[3,5].
  // The notch (x=[3,5], y=[3,5]) is outside the polygon and connects to the map edge.
  setFence({
    {0.0f, 0.0f}, {5.0f, 0.0f}, {5.0f, 3.0f},
    {3.0f, 3.0f}, {3.0f, 5.0f}, {0.0f, 5.0f}});
  updateLayer();

  // Interior cells (inside the L) — must be FREE
  EXPECT_EQ(costmap->getCost(1, 2), nav2_costmap_2d::FREE_SPACE)
    << "Cell inside L bottom arm must be FREE";
  EXPECT_EQ(costmap->getCost(4, 1), nav2_costmap_2d::FREE_SPACE)
    << "Cell inside L right section must be FREE";

  // Cell in notch — exterior, 1 cell from outline, within border_thickness=3 → LETHAL
  EXPECT_EQ(costmap->getCost(4, 4), nav2_costmap_2d::LETHAL_OBSTACLE)
    << "Cell in concave notch must be LETHAL (exterior, reachable from map edge)";

  // Cell just outside right boundary of L, within border_thickness → LETHAL
  EXPECT_EQ(costmap->getCost(6, 1), nav2_costmap_2d::LETHAL_OBSTACLE)
    << "Cell just outside L right edge must be LETHAL";

  // Far exterior cell — outside the L but distance > border_thickness → FREE
  EXPECT_EQ(costmap->getCost(9, 9), nav2_costmap_2d::FREE_SPACE)
    << "Far outside cell must be FREE (distance exceeds border_thickness)";
}

// Standalone test: initialization validation throws on massive border
TEST(GeofenceLayerInitTest, InitRejectsMassiveBorder)
{
  auto node = std::make_shared<TestLifecycleNode>("geofence_throw_border_node");
  node->declare_parameter("gf_border_bad.border_thickness", 50000);

  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  auto tf = nav2::create_transform_buffer(node);

  auto layer = std::make_shared<TestableGeofenceLayer>();
  EXPECT_THROW(
    layer->initialize(&layers, "gf_border_bad", tf.get(), node, nullptr),
    std::runtime_error);
}

// Edge case: 1-row map — exterior seeding side-column loop is a no-op (y+1 < 1 is false).
// Verify no crash and correct rasterization.
TEST(GeofenceLayerSizeTest, SingleRowMap)
{
  auto node = std::make_shared<TestLifecycleNode>("geofence_single_row_node");
  node->declare_parameter("gf_1row.resize_to_fence", false);

  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 1, 1.0, 0.0, 0.0);  // 10 wide, 1 tall
  auto tf = nav2::create_transform_buffer(node);

  auto layer = std::make_shared<TestableGeofenceLayer>();
  layer->initialize(&layers, "gf_1row", tf.get(), node, nullptr);
  layers.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(layer));

  // Rectangle spanning world x=[3,7], y=[0,1] — the full height of the 1-row map
  auto request = std::make_shared<nav2_msgs::srv::SetFence::Request>();
  request->fence.header.frame_id = "map";
  for (const auto & [x, y] : std::vector<std::pair<float, float>>{
    {3.0f, 0.0f}, {7.0f, 0.0f}, {7.0f, 1.0f}, {3.0f, 1.0f}})
  {
    geometry_msgs::msg::Point32 p;
    p.x = x; p.y = y; p.z = 0.0f;
    request->fence.polygon.points.push_back(p);
  }
  auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  ASSERT_NO_THROW(layer->callSetFence(request, response));
  ASSERT_TRUE(response->success);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  costmap->resetMap(0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  double min_x = 1e6, min_y = 1e6, max_x = -1e6, max_y = -1e6;
  ASSERT_NO_THROW(layer->updateBounds(5.0, 0.5, 0.0, &min_x, &min_y, &max_x, &max_y));
  ASSERT_NO_THROW(layer->updateCosts(
    *costmap, 0, 0,
    static_cast<int>(costmap->getSizeInCellsX()),
    static_cast<int>(costmap->getSizeInCellsY())));
}


TEST(GeofenceLayerSizeTest, RollingCostmapDisablesResize)
{
  auto node = std::make_shared<TestLifecycleNode>("gf_rolling_node");
  node->declare_parameter("gf_rolling.resize_to_fence", true);

  nav2_costmap_2d::LayeredCostmap layers("map", true, false);  // true = rolling costmap
  auto tf = nav2::create_transform_buffer(node);

  auto layer = std::make_shared<TestableGeofenceLayer>();
  // Trigger the warning and override resize_to_fence_ to false, allowing init to complete cleanly
  ASSERT_NO_THROW(layer->initialize(&layers, "gf_rolling", tf.get(), node, nullptr));
}

TEST_F(GeofenceLayerTest, TFTransformException)
{
  // Provide a polygon in an unknown frame to trigger lookupTransform failure
  auto request = std::make_shared<nav2_msgs::srv::SetFence::Request>();
  request->fence.header.frame_id = "unknown_frame_xyz";

  geometry_msgs::msg::Point32 p;
  p.x = 0; p.y = 0; p.z = 0;
  request->fence.polygon.points.push_back(p);
  request->fence.polygon.points.push_back(p);
  request->fence.polygon.points.push_back(p);  // 3 points needed

  auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  ASSERT_NO_THROW(geofence_->callSetFence(request, response));

  EXPECT_FALSE(response->success) << "Expected failure for unknown frame";
}

TEST(GeofenceLayerSizeTest, ResizeToFenceActive)
{
  auto node = std::make_shared<TestLifecycleNode>("gf_resize_active");
  node->declare_parameter("gf_resize.resize_to_fence", true);
  node->declare_parameter("gf_resize.border_thickness", 1);

  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto tf = nav2::create_transform_buffer(node);

  auto layer = std::make_shared<TestableGeofenceLayer>();
  layer->initialize(&layers, "gf_resize", tf.get(), node, nullptr);
  layers.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(layer));

  // Set a fence that expands the bounds (from -10 to 20)
  auto request = std::make_shared<nav2_msgs::srv::SetFence::Request>();
  request->fence.header.frame_id = "map";
  for (const auto & [x, y] : std::vector<std::pair<float, float>>{
    {-10.0f, -10.0f}, {20.0f, -10.0f}, {20.0f, 20.0f}, {-10.0f, 20.0f}})
  {
    geometry_msgs::msg::Point32 p;
    p.x = x; p.y = y; p.z = 0.0f;
    request->fence.polygon.points.push_back(p);
  }
  auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  layer->callSetFence(request, response);

  double min_x = 1e6, min_y = 1e6, max_x = -1e6, max_y = -1e6;
  // This will trigger processFence -> resizeMap
  layer->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);

  auto * costmap = layers.getCostmap();
  EXPECT_GT(costmap->getSizeInCellsX(), 10u) << "Costmap should have resized to fit fence";
}

TEST(GeofenceLayerSizeTest, RollingCostmapUpdateWarningThrottling)
{
  auto node = std::make_shared<TestLifecycleNode>("gf_rolling_upd");
  nav2_costmap_2d::LayeredCostmap layers("map", true, false);  // true = rolling costmap
  auto tf = nav2::create_transform_buffer(node);

  auto layer = std::make_shared<TestableGeofenceLayer>();
  layer->initialize(&layers, "gf_rolling_upd", tf.get(), node, nullptr);

  // To hit updateCosts, has_fence_ must be true, and enabled_ must be true
  // Set fence
  auto request = std::make_shared<nav2_msgs::srv::SetFence::Request>();
  request->fence.header.frame_id = "map";
  geometry_msgs::msg::Point32 p;
  p.x = 0; p.y = 0; p.z = 0;
  request->fence.polygon.points.push_back(p);
  request->fence.polygon.points.push_back(p);
  request->fence.polygon.points.push_back(p);
  auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  layer->callSetFence(request, response);

  double min_x = 1e6, min_y = 1e6, max_x = -1e6, max_y = -1e6;
  layer->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);

  // Call updateCosts 11 times to trigger the throttle warning
  nav2_costmap_2d::Costmap2D master;
  for (int i = 0; i < 11; ++i) {
    layer->updateCosts(master, 0, 0, 10, 10);
  }

  // Also cover the warning during dynamic parameter update for rolling
  layer->callUpdate({rclcpp::Parameter("gf_rolling_upd.resize_to_fence", true)});
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
