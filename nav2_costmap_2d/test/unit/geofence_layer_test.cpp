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

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/geofence_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

class TestLifecycleNode : public nav2::LifecycleNode
{
public:
  explicit TestLifecycleNode(const std::string & name)
  : nav2::LifecycleNode(name) {}

  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {return nav2::CallbackReturn::SUCCESS;}
};

class TestableGeofenceLayer : public nav2_costmap_2d::GeofenceLayer
{
public:
  void setFence(
    const std::shared_ptr<nav2_msgs::srv::SetFence::Request> & request,
    const std::shared_ptr<nav2_msgs::srv::SetFence::Response> & response)
  {
    setFenceCallback(std::make_shared<rmw_request_id_t>(), request, response);
  }

  rcl_interfaces::msg::SetParametersResult validate(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    return validateParameterUpdatesCallback(parameters);
  }

  void updateParams(const std::vector<rclcpp::Parameter> & parameters)
  {
    updateParametersCallback(parameters);
  }
};

std::shared_ptr<nav2_msgs::srv::SetFence::Request> makeFenceRequest(
  const std::string & frame_id,
  const std::vector<std::pair<float, float>> & vertices)
{
  auto request = std::make_shared<nav2_msgs::srv::SetFence::Request>();
  request->fence.header.frame_id = frame_id;
  for (const auto & [x, y] : vertices) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    request->fence.polygon.points.push_back(point);
  }
  return request;
}

class GeofenceLayerTest : public ::testing::Test
{
protected:
  GeofenceLayerTest()
  : layers_("map", false, false)
  {
    node_ = std::make_shared<TestLifecycleNode>("geofence_layer_test");
    node_->declare_parameter("transform_tolerance", 0.0);
    node_->declare_parameter("geofence.resize_to_fence", false);
    layers_.resizeMap(10, 10, 1.0, 0.0, 0.0);
    tf_ = nav2::create_transform_buffer(node_);
    geofence_ = std::make_shared<TestableGeofenceLayer>();
    geofence_->initialize(&layers_, "geofence", tf_.get(), node_, nullptr);
    layers_.addPlugin(geofence_);
  }

  void setFence(const std::vector<std::pair<float, float>> & vertices)
  {
    auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
    geofence_->setFence(makeFenceRequest("map", vertices), response);
    ASSERT_TRUE(response->success) << response->message;
  }

  void updateMap()
  {
    layers_.updateMap(5.0, 5.0, 0.0);
  }

  nav2_costmap_2d::Costmap2D * costmap()
  {
    return layers_.getCostmap();
  }

  std::shared_ptr<TestLifecycleNode> node_;
  nav2::TransformBuffer::SharedPtr tf_;
  nav2_costmap_2d::LayeredCostmap layers_;
  std::shared_ptr<TestableGeofenceLayer> geofence_;
};

TEST_F(GeofenceLayerTest, LeavesCostmapUnchangedWithoutFence)
{
  EXPECT_TRUE(geofence_->isCurrent());
  updateMap();
  EXPECT_EQ(costmap()->getCost(4, 4), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(GeofenceLayerTest, MatchSizePreservesFence)
{
  setFence({{2.0f, 2.0f}, {7.0f, 2.0f}, {7.0f, 7.0f}, {2.0f, 7.0f}});
  updateMap();
  EXPECT_EQ(costmap()->getCost(4, 4), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(costmap()->getCost(2, 2), nav2_costmap_2d::LETHAL_OBSTACLE);

  geofence_->matchSize();
  updateMap();
  EXPECT_EQ(costmap()->getCost(4, 4), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(costmap()->getCost(2, 2), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(GeofenceLayerTest, MarksExteriorAndPerimeterLethal)
{
  setFence({{2.0f, 2.0f}, {7.0f, 2.0f}, {7.0f, 7.0f}, {2.0f, 7.0f}});
  updateMap();

  EXPECT_EQ(costmap()->getCost(4, 4), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(costmap()->getCost(2, 2), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(costmap()->getCost(1, 4), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(costmap()->getCost(9, 9), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(GeofenceLayerTest, ClearsFenceThroughCostmapUpdate)
{
  setFence({{2.0f, 2.0f}, {7.0f, 2.0f}, {7.0f, 7.0f}, {2.0f, 7.0f}});
  updateMap();
  EXPECT_EQ(costmap()->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);

  setFence({});
  updateMap();
  EXPECT_EQ(costmap()->getCost(0, 0), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(GeofenceLayerTest, ReplacingFenceUpdatesMap)
{
  setFence({{2.0f, 2.0f}, {7.0f, 2.0f}, {7.0f, 7.0f}, {2.0f, 7.0f}});
  updateMap();
  EXPECT_EQ(costmap()->getCost(4, 4), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(costmap()->getCost(3, 4), nav2_costmap_2d::FREE_SPACE);

  // Shrink the fence; cell (3,4) was interior but is now on fence 2's perimeter.
  setFence({{3.0f, 3.0f}, {6.0f, 3.0f}, {6.0f, 6.0f}, {3.0f, 6.0f}});
  updateMap();
  EXPECT_EQ(costmap()->getCost(4, 4), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(costmap()->getCost(3, 4), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(GeofenceLayerTest, SupportsConcaveAndClippedPolygons)
{
  setFence({
    {-2.0f, 0.0f}, {6.0f, 0.0f}, {6.0f, 3.0f},
    {3.0f, 3.0f}, {3.0f, 8.0f}, {-2.0f, 8.0f}});
  updateMap();

  EXPECT_EQ(costmap()->getCost(1, 1), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(costmap()->getCost(4, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(GeofenceLayerTest, RejectsInvalidFenceGeometry)
{
  // Too few vertices.
  auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  geofence_->setFence(makeFenceRequest("map", {{1.0f, 1.0f}, {2.0f, 2.0f}}), response);
  EXPECT_FALSE(response->success);

  // Collinear vertices (zero area).
  response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  geofence_->setFence(
    makeFenceRequest("map", {{1.0f, 1.0f}, {2.0f, 2.0f}, {3.0f, 3.0f}}), response);
  EXPECT_FALSE(response->success);

  // Self-intersecting hourglass: (1,1)-(5,5)-(5,1)-(1,5) has non-zero area but crossing edges.
  response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  geofence_->setFence(
    makeFenceRequest("map", {{1.0f, 1.0f}, {5.0f, 5.0f}, {5.0f, 1.0f}, {1.0f, 5.0f}}), response);
  EXPECT_FALSE(response->success);

  // Non-finite coordinate.
  response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  geofence_->setFence(
    makeFenceRequest(
      "map",
      {{1.0f, 1.0f}, {std::numeric_limits<float>::infinity(), 2.0f}, {3.0f, 3.0f}}),
    response);
  EXPECT_FALSE(response->success);
}

TEST_F(GeofenceLayerTest, EnabledFalseSkipsFenceCosts)
{
  setFence({{2.0f, 2.0f}, {7.0f, 2.0f}, {7.0f, 7.0f}, {2.0f, 7.0f}});
  updateMap();
  EXPECT_EQ(costmap()->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);

  geofence_->updateParams({rclcpp::Parameter("geofence.enabled", false)});
  updateMap();
  EXPECT_EQ(costmap()->getCost(0, 0), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(GeofenceLayerTest, UsesGlobalFrameTransform)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = node_->now();
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 2.0;
  transform.transform.translation.y = 2.0;
  transform.transform.rotation.w = 1.0;
  tf_->setTransform(transform, "test", false);

  // Zero stamp -> TimePointZero path.
  auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  geofence_->setFence(
    makeFenceRequest("base_link", {{0.0f, 0.0f}, {4.0f, 0.0f}, {4.0f, 4.0f}, {0.0f, 4.0f}}),
    response);
  ASSERT_TRUE(response->success) << response->message;
  updateMap();
  EXPECT_EQ(costmap()->getCost(3, 3), nav2_costmap_2d::FREE_SPACE);

  // Real stamp -> timestamped lookup path.
  auto request = makeFenceRequest(
    "base_link", {{0.0f, 0.0f}, {4.0f, 0.0f}, {4.0f, 4.0f}, {0.0f, 4.0f}});
  request->fence.header.stamp = transform.header.stamp;
  response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  geofence_->setFence(request, response);
  EXPECT_TRUE(response->success) << response->message;
}

TEST_F(GeofenceLayerTest, RejectsFenceInUnknownFrame)
{
  auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  geofence_->setFence(
    makeFenceRequest(
      "unknown_frame",
      {{0.0f, 0.0f}, {4.0f, 0.0f}, {4.0f, 4.0f}, {0.0f, 4.0f}}),
    response);
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->message.empty());
}

TEST_F(GeofenceLayerTest, DynamicParameterValidationAndUpdates)
{
  auto result = geofence_->validate(
    {rclcpp::Parameter("geofence.fence_polygon", "[[1,1],[2,1],[1,2]]")});
  EXPECT_FALSE(result.successful);

  result = geofence_->validate({rclcpp::Parameter("geofence.enabled", 123)});
  EXPECT_FALSE(result.successful);

  result = geofence_->validate({rclcpp::Parameter("geofence.resize_to_fence", "true")});
  EXPECT_FALSE(result.successful);

  result = geofence_->validate({rclcpp::Parameter("geofence.unrelated", "value")});
  EXPECT_TRUE(result.successful);

  geofence_->updateParams({
    rclcpp::Parameter("geofence.resize_to_fence", false),
    rclcpp::Parameter("geofence.unrelated", 42)});
}

TEST_F(GeofenceLayerTest, LifecycleAndReset)
{
  geofence_->activate();
  geofence_->deactivate();
  geofence_->deactivate();

  geofence_->reset();
  EXPECT_FALSE(geofence_->isCurrent());
}

TEST_F(GeofenceLayerTest, IgnoresFenceOutsideCostmap)
{
  setFence({{20.0f, 20.0f}, {30.0f, 20.0f}, {30.0f, 30.0f}, {20.0f, 30.0f}});
  updateMap();
  EXPECT_EQ(costmap()->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(GeofenceLayerSizeTest, ResizesForEachFenceUpdate)
{
  auto node = std::make_shared<TestLifecycleNode>("geofence_resize_test");
  node->declare_parameter("transform_tolerance", 0.0);
  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto tf = nav2::create_transform_buffer(node);
  auto layer = std::make_shared<TestableGeofenceLayer>();
  layer->initialize(&layers, "geofence", tf.get(), node, nullptr);
  layers.addPlugin(layer);

  auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  layer->setFence(
    makeFenceRequest("map", {{-10.0f, -10.0f}, {20.0f, -10.0f}, {20.0f, 20.0f}, {-10.0f, 20.0f}}),
    response);
  ASSERT_TRUE(response->success);
  layers.updateMap(0.0, 0.0, 0.0);
  const unsigned int first_size_x = layers.getCostmap()->getSizeInCellsX();

  response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  layer->setFence(
    makeFenceRequest("map", {{-20.0f, -20.0f}, {30.0f, -20.0f}, {30.0f, 30.0f}, {-20.0f, 30.0f}}),
    response);
  ASSERT_TRUE(response->success);
  layers.updateMap(0.0, 0.0, 0.0);
  EXPECT_GT(layers.getCostmap()->getSizeInCellsX(), first_size_x);
}

TEST(GeofenceLayerSizeTest, ClearRestoresInitialDimensions)
{
  auto node = std::make_shared<TestLifecycleNode>("geofence_clear_test");
  node->declare_parameter("transform_tolerance", 0.0);
  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto tf = nav2::create_transform_buffer(node);
  auto layer = std::make_shared<TestableGeofenceLayer>();
  layer->initialize(&layers, "geofence", tf.get(), node, nullptr);
  layers.addPlugin(layer);

  auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  layer->setFence(
    makeFenceRequest("map", {{-10.0f, -10.0f}, {20.0f, -10.0f}, {20.0f, 20.0f}, {-10.0f, 20.0f}}),
    response);
  ASSERT_TRUE(response->success);
  layers.updateMap(0.0, 0.0, 0.0);
  EXPECT_GT(layers.getCostmap()->getSizeInCellsX(), 10u);

  response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  layer->setFence(makeFenceRequest("map", {}), response);
  ASSERT_TRUE(response->success);
  layers.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(layers.getCostmap()->getSizeInCellsX(), 10u);
  EXPECT_EQ(layers.getCostmap()->getSizeInCellsY(), 10u);
  EXPECT_EQ(layers.getCostmap()->getCost(0, 0), nav2_costmap_2d::FREE_SPACE);
}

TEST(GeofenceLayerSizeTest, RejectsOversizedFence)
{
  auto node = std::make_shared<TestLifecycleNode>("geofence_overflow_test");
  node->declare_parameter("transform_tolerance", 0.0);
  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto tf = nav2::create_transform_buffer(node);
  auto layer = std::make_shared<TestableGeofenceLayer>();
  layer->initialize(&layers, "geofence", tf.get(), node, nullptr);
  layers.addPlugin(layer);

  const float kHuge = 1e5f;
  auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  layer->setFence(
    makeFenceRequest(
      "map",
      {{-kHuge, -kHuge}, {kHuge, -kHuge}, {kHuge, kHuge}, {-kHuge, kHuge}}),
    response);
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->message.empty());
  EXPECT_NO_THROW(layers.updateMap(0.0, 0.0, 0.0));
  EXPECT_EQ(layers.getCostmap()->getSizeInCellsX(), 10u);
}

TEST(GeofenceLayerSizeTest, RejectsRollingCostmap)
{
  auto node = std::make_shared<TestLifecycleNode>("geofence_rolling_test");
  node->declare_parameter("transform_tolerance", 0.0);
  nav2_costmap_2d::LayeredCostmap layers("map", true, false);
  auto tf = nav2::create_transform_buffer(node);
  auto layer = std::make_shared<TestableGeofenceLayer>();
  EXPECT_THROW(layer->initialize(&layers, "geofence", tf.get(), node, nullptr), std::runtime_error);
}

TEST(GeofenceLayerStartupTest, LoadsFenceFromParameter)
{
  auto node = std::make_shared<TestLifecycleNode>("geofence_startup_test");
  node->declare_parameter("transform_tolerance", 0.0);
  node->declare_parameter("geofence.resize_to_fence", false);
  node->declare_parameter("geofence.fence_polygon", "[[2,2],[7,2],[7,7],[2,7]]");
  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto tf = nav2::create_transform_buffer(node);
  auto layer = std::make_shared<TestableGeofenceLayer>();
  layer->initialize(&layers, "geofence", tf.get(), node, nullptr);
  layers.addPlugin(layer);
  layers.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(layers.getCostmap()->getCost(4, 4), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(layers.getCostmap()->getCost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(GeofenceLayerStartupTest, ThrowsForInvalidFenceParameter)
{
  auto node = std::make_shared<TestLifecycleNode>("geofence_invalid_test");
  node->declare_parameter("transform_tolerance", 0.0);
  node->declare_parameter("geofence.fence_polygon", "not_a_valid_polygon");
  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto tf = nav2::create_transform_buffer(node);
  auto layer = std::make_shared<TestableGeofenceLayer>();
  EXPECT_THROW(
    layer->initialize(&layers, "geofence", tf.get(), node, nullptr),
    std::runtime_error);
}

TEST(GeofenceLayerSizeTest, ResizesOnlyWhenAabbChanges)
{
  auto node = std::make_shared<TestLifecycleNode>("geofence_aabb_test");
  node->declare_parameter("transform_tolerance", 0.0);
  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto tf = nav2::create_transform_buffer(node);
  auto layer = std::make_shared<TestableGeofenceLayer>();
  layer->initialize(&layers, "geofence", tf.get(), node, nullptr);
  layers.addPlugin(layer);

  auto response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  layer->setFence(
    makeFenceRequest(
      "map", {{-10.0f, -10.0f}, {20.0f, -10.0f}, {20.0f, 20.0f}, {-10.0f, 20.0f}}),
    response);
  ASSERT_TRUE(response->success);
  layers.updateMap(0.0, 0.0, 0.0);
  const unsigned int size_x = layers.getCostmap()->getSizeInCellsX();

  response = std::make_shared<nav2_msgs::srv::SetFence::Response>();
  layer->setFence(
    makeFenceRequest(
      "map", {{5.0f, -10.0f}, {20.0f, 5.0f}, {5.0f, 20.0f}, {-10.0f, 5.0f}}),
    response);
  ASSERT_TRUE(response->success);
  layers.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(layers.getCostmap()->getSizeInCellsX(), size_x);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
