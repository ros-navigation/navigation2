// Copyright 2025 Kudan Ltd

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

#include <cstdio>
#include <memory>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "../testing_helper.hpp"
#include "tf2_ros/buffer.hpp"


class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
  explicit TestLifecycleNode(const std::string & name)
  : nav2_util::LifecycleNode(name)
  {
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onError(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }
};

class ObstacleLayerTest : public ::testing::Test
{
public:
  explicit ObstacleLayerTest(double resolution = 0.1)
  : layers_("frame", false, false)
  {
    node_ = std::make_shared<TestLifecycleNode>("obstacle_cell_distance_test_node");
    node_->declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
    node_->declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
    node_->declare_parameter("use_maximum", rclcpp::ParameterValue(false));
    node_->declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
    node_->declare_parameter(
      "unknown_cost_value",
      rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
    node_->declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
    node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
    node_->declare_parameter("observation_sources", rclcpp::ParameterValue(std::string("")));

    // 20x20 cells with origin at (0, 0)
    layers_.resizeMap(20, 20, resolution, 0, 0);
    tf2_ros::Buffer tf(node_->get_clock());
    addObstacleLayer(layers_, tf, node_, obstacle_layer_);
  }

  ~ObstacleLayerTest() {}

  void update()
  {
    double min_x, min_y, max_x, max_y;
    obstacle_layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  }

  void printMap()
  {
    printf("Printing map:\n");
    for (unsigned int y = 0; y < obstacle_layer_->getSizeInCellsY(); y++) {
      for (unsigned int x = 0; x < obstacle_layer_->getSizeInCellsX(); x++) {
        printf("%4d", static_cast<int>(obstacle_layer_->getCost(x, y)));
      }
      printf("\n");
    }
  }

protected:
  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> obstacle_layer_;

private:
  std::shared_ptr<TestLifecycleNode> node_;
  nav2_costmap_2d::LayeredCostmap layers_;
};

class ObstacleLayerFineResolutionTest : public ObstacleLayerTest
{
public:
  ObstacleLayerFineResolutionTest()
  : ObstacleLayerTest(0.05) {}
};

/**
 * Test that a point within cell_max_range is marked as obstacle
 */
TEST_F(ObstacleLayerTest, testPointWithinCellMaxRange)
{
  // Add observation: point at (0.55, 0.0)
  // obstacle_max_range = 1.0m
  addObservation(
    obstacle_layer_, 0.55, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 100.0, 0.0, 1.0, 0.0);
  update();

  unsigned int mx, my;
  obstacle_layer_->worldToMap(0.55, 0.0, mx, my);
  unsigned char cost = obstacle_layer_->getCost(mx, my);

  ASSERT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test that a point beyond cell_max_range is NOT marked as obstacle
 */
TEST_F(ObstacleLayerTest, testPointBeyondCellMaxRange)
{
  // Add observation: point at (1.55, 0.0)
  // obstacle_max_range = 1.0m
  addObservation(
    obstacle_layer_, 1.55, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 100.0, 0.0, 1.0, 0.0);
  update();

  unsigned int mx, my;
  obstacle_layer_->worldToMap(1.55, 0.0, mx, my);
  unsigned char cost = obstacle_layer_->getCost(mx, my);

  ASSERT_NE(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test that a point below cell_min_range is NOT marked as obstacle
 */
TEST_F(ObstacleLayerTest, testPointWithinCellMinRange)
{
  // Add observation: point at (0.35, 0.0)
  // obstacle_min_range = 0.5m, obstacle_max_range = 10.0m
  addObservation(
    obstacle_layer_, 0.35, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 100.0, 0.0, 10.0, 0.5);
  update();

  unsigned int mx, my;
  obstacle_layer_->worldToMap(0.35, 0.0, mx, my);
  unsigned char cost = obstacle_layer_->getCost(mx, my);

  ASSERT_NE(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test that points within diagonal distance are marked as obstacles
 */
TEST_F(ObstacleLayerTest, testDiagonalDistanceWithinRange)
{
  // Add observation: point at (0.701, 0.701)
  // obstacle_max_range = 1.0m
  // obstacle range = sqrt(7^2 + 7^2) = 9.9 cells
  addObservation(
    obstacle_layer_, 0.701, 0.701, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 100.0, 0.0, 1.0, 0.0);
  update();
  unsigned int mx, my;
  obstacle_layer_->worldToMap(0.701, 0.701, mx, my);
  unsigned char cost = obstacle_layer_->getCost(mx, my);

  ASSERT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test that point beyond obstacle max range but in same cell as max range is
 * marked as obstacle.
 */
TEST_F(ObstacleLayerTest, testPointBeyondRangeButInSameCellAsMaxRange) {
  // Observation at (0.79, 0.0)
  // obstacle_max_range = 0.72m
  // obstacle range = 0.79m, > 0.72m but at same cell distance as max range so
  // should be marked as obstacle, since it could be cleared by raytracing with
  // the same max range.
  addObservation(
    obstacle_layer_, 0.79, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 100.0, 0.0, 0.72, 0.0);
  update();

  unsigned int mx, my;
  obstacle_layer_->worldToMap(0.79, 0.0, mx, my);
  unsigned char cost = obstacle_layer_->getCost(mx, my);

  ASSERT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test diagonal distance beyond range
 */
TEST_F(ObstacleLayerTest, testDiagonalDistanceBeyondRange)
{
  // Add observation: point at (1.0, 1.0)
  // obstacle_max_range = 1.2m
  // Distance: sqrt(10^2 + 10^2) = 14.14 cells, 14 cells > 12 cells, so should
  // NOT be marked
  addObservation(
    obstacle_layer_, 1.0, 1.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 100.0, 0.0, 1.2, 0.0);
  update();

  unsigned int mx, my;
  obstacle_layer_->worldToMap(1.0, 1.0, mx, my);
  unsigned char cost = obstacle_layer_->getCost(mx, my);

  ASSERT_NE(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test edge case: point exactly at cell_max_range boundary
 */
TEST_F(ObstacleLayerTest, testPointAtCellMaxRangeBoundary)
{
  // Add observation: point at (1.0, 0.0)
  // obstacle_max_range = 1.0m
  addObservation(
    obstacle_layer_, 1.0, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 100.0, 0.0, 1.0, 0.0);
  update();
  unsigned int mx, my;
  obstacle_layer_->worldToMap(1.0, 0.0, mx, my);
  unsigned char cost = obstacle_layer_->getCost(mx, my);

  ASSERT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test edge case: point exactly at cell_min_range boundary
 */
TEST_F(ObstacleLayerTest, testPointAtCellMinRangeBoundary)
{
  // Add observation: point at (0.5, 0.0)
  // obstacle_min_range = 0.5m
  addObservation(
    obstacle_layer_, 0.5, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 100.0, 0.0, 10.0, 0.5);
  update();

  unsigned int mx, my;
  obstacle_layer_->worldToMap(0.5, 0.0, mx, my);
  unsigned char cost = obstacle_layer_->getCost(mx, my);

  ASSERT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test with different resolution
 */
TEST_F(ObstacleLayerFineResolutionTest, testDifferentResolution)
{
  // Add observation: point at (0.52, 0.28)
  // Resolution: 0.05m/cell
  // Cell index: (10, 5)
  addObservation(
    obstacle_layer_, 0.52, 0.28, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 100.0, 0.0, 1.0, 0.0);
  update();

  unsigned char cost = obstacle_layer_->getCost(10, 5);
  ASSERT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test with origin offset
 */
TEST_F(ObstacleLayerTest, testOriginOffset)
{
  // Add observation: point at (1.75, 1.85) with origin at (0.5, 1.5)
  // obstacle_max_range = 2.0m
  addObservation(
    obstacle_layer_, 1.75, 1.85, MAX_Z / 2, 0.5, 1.5, MAX_Z / 2,
    true, true, 100.0, 0.0, 2.0, 0.0);
  update();

  unsigned int mx, my;
  obstacle_layer_->worldToMap(1.75, 1.85, mx, my);
  unsigned char cost = obstacle_layer_->getCost(mx, my);
  ASSERT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test that raytracing clears cells along the path and the endpoint is marked
 * as obstacle.
 */
TEST_F(ObstacleLayerTest, testRaytraceClearsPathAndMarksEndpoint)
{
  // Mark a few cells along the path as obstacles, excluding the endpoint
  for (unsigned int x = 0; x < 4; ++x) {
    obstacle_layer_->setCost(x, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  // Add observation at (0.85, 0.0)
  addObservation(
    obstacle_layer_, 0.85, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 2.0, 0.0, 2.0, 0.0);
  update();

  unsigned int mx_end;
  unsigned int my_end;
  obstacle_layer_->worldToMap(0.85, 0.0, mx_end, my_end);

  // Check that cells before endpoint are cleared (FREE_SPACE)
  for (unsigned int x = 0; x < mx_end; ++x) {
    unsigned char cost = obstacle_layer_->getCost(x, 0);
    ASSERT_EQ(cost, nav2_costmap_2d::FREE_SPACE);
  }

  // Check that the endpoint cell is LETHAL_OBSTACLE
  unsigned char cost_endpoint = obstacle_layer_->getCost(mx_end, my_end);
  ASSERT_EQ(cost_endpoint, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test diagonal raytracing does not clear endpoint
 */
TEST_F(ObstacleLayerTest, testDiagonalRaytraceDoesNotClearEndpoint)
{
  // Mark the endpoint as obstacle
  unsigned int mx_end, my_end;
  obstacle_layer_->worldToMap(0.75, 0.75, mx_end, my_end);
  obstacle_layer_->setCost(mx_end, my_end, nav2_costmap_2d::LETHAL_OBSTACLE);
  // Mark a cell along the diagonal path
  unsigned int mx_mid, my_mid;
  obstacle_layer_->worldToMap(0.45, 0.45, mx_mid, my_mid);
  obstacle_layer_->setCost(mx_mid, my_mid, nav2_costmap_2d::LETHAL_OBSTACLE);

  // Add observation at (0.75, 0.75)
  addObservation(
    obstacle_layer_, 0.75, 0.75, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 2.0, 0.0, 100.0, 0.0);
  update();

  // The endpoint cell should still be LETHAL_OBSTACLE
  unsigned char cost_endpoint = obstacle_layer_->getCost(mx_end, my_end);
  ASSERT_EQ(cost_endpoint, nav2_costmap_2d::LETHAL_OBSTACLE);

  // The intermediate cell should be cleared
  unsigned char cost_mid = obstacle_layer_->getCost(mx_mid, my_mid);
  ASSERT_EQ(cost_mid, nav2_costmap_2d::FREE_SPACE);
}

/**
 * Test diagonal clearing up to max range
 */
TEST_F(ObstacleLayerTest, testClearDiagonalDistance) {
  // Mark all points as obstacles
  for (unsigned int x = 0; x < obstacle_layer_->getSizeInCellsX(); x++) {
    for (unsigned int y = 0; y < obstacle_layer_->getSizeInCellsY(); y++) {
      obstacle_layer_->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  // Add observation at (1.55, 1.55) with max clearing range of 1.0m
  // This should clear the diagonal distance up to range of 1.0m, cells (0, 0)
  // to (7, 7)
  addObservation(
    obstacle_layer_, 1.55, 1.55, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 1.0, 0.0, 100.0, 0.0);
  update();

  for (unsigned int i = 0; i < 8; i++) {
    ASSERT_EQ(obstacle_layer_->getCost(i, i), nav2_costmap_2d::FREE_SPACE);
  }
  ASSERT_EQ(countValues(*obstacle_layer_, nav2_costmap_2d::FREE_SPACE), 8);
  ASSERT_EQ(
    countValues(*obstacle_layer_, nav2_costmap_2d::LETHAL_OBSTACLE),
    20 * 20 - 8);
}
