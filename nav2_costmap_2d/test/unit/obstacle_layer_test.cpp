#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "../testing_helper.hpp"
#include "tf2_ros/buffer.hpp"

#include <cstdio>
#include <memory>
#include <string>
#include <gtest/gtest.h>


class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

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

class ObstacleLayerTest : public ::testing::Test
{
public:
  ObstacleLayerTest(double resolution = 0.1)
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

    layers_.resizeMap(20, 20, resolution, 0, 0); // 20x20 cells
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
  // Add observation: point at (0.5, 0.0)
  // obstacle_max_range = 1.0m
  addObservation(obstacle_layer_, 0.55, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
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
  // Add observation: point at (1.5, 0.0)
  // obstacle_max_range = 1.0m
  addObservation(obstacle_layer_, 1.55, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
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
  // Add observation: point at (0.3, 0.0)
  // obstacle_min_range = 0.5m, obstacle_max_range = 10.0m
  addObservation(obstacle_layer_, 0.35, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
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
  // Add observation: point at (0.7, 0.7)
  // obstacle_max_range = 1.0m
  // Cell distance: max(|7-0|, |7-0|) = 7 cells < 10 cells, so should be marked
  addObservation(obstacle_layer_, 0.75, 0.75, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 100.0, 0.0, 1.0, 0.0);
  update();

  unsigned int mx, my;
  obstacle_layer_->worldToMap(0.75, 0.75, mx, my);
  unsigned char cost = obstacle_layer_->getCost(mx, my);

  ASSERT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test diagonal distance beyond range
 */
TEST_F(ObstacleLayerTest, testDiagonalDistanceBeyondRange)
{
  // Add observation: point at (1.2, 1.2)
  // obstacle_max_range = 1.0m
  // Cell distance: max(|12-0|, |12-0|) = 12 cells > 10 cells, so should NOT be marked
  addObservation(obstacle_layer_, 1.25, 1.25, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 100.0, 0.0, 1.0, 0.0);
  update();

  unsigned int mx, my;
  obstacle_layer_->worldToMap(1.25, 1.25, mx, my);
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
  addObservation(obstacle_layer_, 1.0, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
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
  addObservation(obstacle_layer_, 0.5, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 100.0, 0.0, 10.0, 0.5);
  update();

  unsigned int mx, my;
  obstacle_layer_->worldToMap(0.5, 0.0, mx, my);
  unsigned char cost = obstacle_layer_->getCost(mx, my);

  ASSERT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test asymmetric distance: point with different delta_x and delta_y
 */
TEST_F(ObstacleLayerTest, testAsymmetricDistance)
{
  // Add observation: point at (0.8, 0.3)
  // obstacle_max_range = 1.0m
  // Cell distance: max(|8-0|, |3-0|) = max(8, 3) = 8 cells < 10 cells, so should be marked
  addObservation(obstacle_layer_, 0.85, 0.55, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 100.0, 0.0, 1.0, 0.0);
  update();

  unsigned int mx, my;
  obstacle_layer_->worldToMap(0.85, 0.55, mx, my);
  unsigned char cost = obstacle_layer_->getCost(mx, my);

  ASSERT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test with different resolution
 */
TEST_F(ObstacleLayerFineResolutionTest, testDifferentResolution)
{
  // Add observation: point at (0.5, 0.0)
  // obstacle_max_range = 0.5m = 10 cells at 0.05m resolution
  addObservation(obstacle_layer_, 0.52, 0.32, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 100.0, 0.0, 0.5, 0.0);
  update();

  // x = 0.5 / 0.05 = 10; y = 0.3 / 0.05 = 6;
  unsigned char cost = obstacle_layer_->getCost(10, 6);
  ASSERT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test with origin offset
 */
TEST_F(ObstacleLayerTest, testOriginOffset)
{
  // Add observation: point at (5.5, 2.5) with origin at (5.0, 2.0)
  // obstacle_max_range = 2.0m
  addObservation(obstacle_layer_, 1.75, 1.85, MAX_Z / 2, 0.5, 1.5, MAX_Z / 2,
    true, true, 100.0, 0.0, 2.0, 0.0);
  update();

  unsigned int mx, my;
  obstacle_layer_->worldToMap(1.75, 1.85, mx, my);
  unsigned char cost = obstacle_layer_->getCost(mx, my);

  ASSERT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test that raytracing does NOT clear the cell containing the observation endpoint
 * raytrace_max_range > obstacle_max_range
 */
TEST_F(ObstacleLayerTest, testRaytraceDoesNotClearEndpointCell)
{
  // First, mark the endpoint cell and some intermediate cells as obstacles
  unsigned int mx_end;
  unsigned int my_end;
  obstacle_layer_->worldToMap(1.05, 0.0, mx_end, my_end);
  for (unsigned int x = 0; x <= mx_end; ++x) {
    obstacle_layer_->setCost(x, my_end, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  // Add observation at (1.01, 0.0)
  // This is beyond the obstacle_max_range of 0.5m so should not be marked.
  // raytrace_max_range = 2.0m
  // So raytrace should clear cells up to but NOT including the endpoint
  addObservation(obstacle_layer_, 1.05, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 2.0, 0.0, 0.5, 0.0);
  update();

  // The endpoint cell should still be LETHAL_OBSTACLE (not cleared)
  unsigned char cost_endpoint = obstacle_layer_->getCost(mx_end, my_end);
  ASSERT_EQ(cost_endpoint, nav2_costmap_2d::LETHAL_OBSTACLE);

  // The intermediate cells should be cleared (FREE_SPACE)
  for (unsigned int x = 0; x < mx_end; ++x) {
    unsigned char cost_mid = obstacle_layer_->getCost(x, my_end);
    ASSERT_EQ(cost_mid, nav2_costmap_2d::FREE_SPACE);
  }
}

/**
 * Test that raytracing clears cells along the path and the endpoint is marked as LETHAL_OBSTACLE
 */
TEST_F(ObstacleLayerTest, testRaytraceClearsPathAndMarksEndpoint)
{
  // Mark a few cells along the path as obstacles, excluding the endpoint
  for (unsigned int x = 0; x < 4; ++x) {
    obstacle_layer_->setCost(x, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  // Add observation at (0.85, 0.0)
  addObservation(obstacle_layer_, 0.85, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
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

  // Add observation at (0.7, 0.7)
  addObservation(obstacle_layer_, 0.75, 0.75, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
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
 * Test edge case: observation very close to origin
 * Resolution: 0.1m/cell
 * Origin at (0.0, 0.0), observation at (0.09, 0.0) - same cell as origin
 * This means no raytracing should occur
 */
TEST_F(ObstacleLayerTest, testRaytraceWithObservationCloseToOrigin)
{
  // Mark all points as obstacles
  for (unsigned int x = 0; x < obstacle_layer_->getSizeInCellsX(); x++) {
    for (unsigned int y = 0; y < obstacle_layer_->getSizeInCellsY(); y++) {
      obstacle_layer_->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  // Add observation at (0.09, 0.0)
  addObservation(obstacle_layer_, 0.09, 0.0, MAX_Z / 2, 0.0, 0.0, MAX_Z / 2,
    true, true, 2.0, 0.0, 100.0, 0.0);
  update();

  // All points should still be LETHAL_OBSTACLE
  for (unsigned int x = 0; x < obstacle_layer_->getSizeInCellsX(); x++) {
    for (unsigned int y = 0; y < obstacle_layer_->getSizeInCellsY(); y++) {
      unsigned char cost = obstacle_layer_->getCost(x, y);
      ASSERT_EQ(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}
