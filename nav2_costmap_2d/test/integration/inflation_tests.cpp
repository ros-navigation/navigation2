/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author David Lu!!
 * Test harness for InflationLayer for Costmap2D
 */
#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "../testing_helper.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

using geometry_msgs::msg::Point;

class TestNode : public ::testing::Test
{
public:
  TestNode() {}

  ~TestNode() {}

  std::vector<Point> setRadii(
    nav2_costmap_2d::LayeredCostmap & layers,
    double length, double width);

  void validatePointInflation(
    unsigned int mx, unsigned int my,
    nav2_costmap_2d::Costmap2D * costmap,
    std::shared_ptr<nav2_costmap_2d::InflationLayer> & ilayer,
    double inflation_radius);

  void initNode(std::vector<rclcpp::Parameter> parameters);
  void initNode(double inflation_radius);

  void waitForMap(std::shared_ptr<nav2_costmap_2d::StaticLayer> & slayer);

protected:
  nav2::LifecycleNode::SharedPtr node_;
};

std::vector<Point> TestNode::setRadii(
  nav2_costmap_2d::LayeredCostmap & layers,
  double length, double width)
{
  std::vector<Point> polygon;
  Point p;
  p.x = width;
  p.y = length;
  polygon.push_back(p);
  p.x = width;
  p.y = -length;
  polygon.push_back(p);
  p.x = -width;
  p.y = -length;
  polygon.push_back(p);
  p.x = -width;
  p.y = length;
  polygon.push_back(p);
  layers.setFootprint(polygon);

  return polygon;
}

void TestNode::waitForMap(std::shared_ptr<nav2_costmap_2d::StaticLayer> & slayer)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_->get_node_base_interface());
  while (!slayer->isCurrent()) {
    executor.spin_some();
  }
}

// Test that a single point gets inflated properly
void TestNode::validatePointInflation(
  unsigned int mx, unsigned int my,
  nav2_costmap_2d::Costmap2D * costmap,
  std::shared_ptr<nav2_costmap_2d::InflationLayer> & ilayer,
  double inflation_radius)
{
  // Validate that inflation costs are correctly applied based on Euclidean distance
  // from the obstacle at (mx, my)

  const unsigned int size_x = costmap->getSizeInCellsX();
  const unsigned int size_y = costmap->getSizeInCellsY();
  const int inflation_cells = static_cast<int>(std::ceil(inflation_radius /
    costmap->getResolution()));

  // Check all cells within the inflation radius
  for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
      const int x = static_cast<int>(mx) + dx;
      const int y = static_cast<int>(my) + dy;

      // Skip cells outside the map
      if (x < 0 || y < 0 || x >= static_cast<int>(size_x) || y >= static_cast<int>(size_y)) {
        continue;
      }

      // Calculate Euclidean distance in cells
      const double dist = std::hypot(dx, dy);

      // Get actual cost from costmap
      const unsigned char actual_cost = costmap->getCost(x, y);

      if (dist <= inflation_radius / costmap->getResolution()) {
        // Within inflation radius - should have inflation cost
        const unsigned char expected_cost = ilayer->computeCost(dist);

        // The actual cost should be at least the expected cost
        // (it could be higher if there are multiple nearby obstacles)
        ASSERT_GE(actual_cost, expected_cost)
          << "At (" << x << ", " << y << ") distance " << dist
          << " from obstacle at (" << mx << ", " << my << "): "
          << "actual cost " << static_cast<int>(actual_cost)
          << " < expected cost " << static_cast<int>(expected_cost);

        // Verify the cost is not free space (unless it's exactly at inflation radius edge)
        if (dist < inflation_radius / costmap->getResolution() - 0.5) {
          ASSERT_GT(actual_cost, nav2_costmap_2d::FREE_SPACE)
            << "Cell at (" << x << ", " << y << ") should be inflated but has FREE_SPACE cost";
        }
      } else {
        // Outside inflation radius - cost should be free space or affected by other obstacles
        // We don't check this strictly as other obstacles may affect the cost
      }
    }
  }
}

void TestNode::initNode(std::vector<rclcpp::Parameter> parameters)
{
  auto options = rclcpp::NodeOptions();
  options.parameter_overrides(parameters);

  node_ = std::make_shared<nav2::LifecycleNode>(
    "inflation_test_node", "", options);

  // Declare non-plugin specific costmap parameters
  node_->declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
  node_->declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
  node_->declare_parameter("use_maximum", rclcpp::ParameterValue(false));
  node_->declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
  node_->declare_parameter("inscribed_obstacle_cost_value", rclcpp::ParameterValue(99));
  node_->declare_parameter(
    "unknown_cost_value",
    rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
  node_->declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
  node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
  node_->declare_parameter("observation_sources", rclcpp::ParameterValue(std::string("")));
}

void TestNode::initNode(double inflation_radius)
{
  std::vector<rclcpp::Parameter> parameters;
  // Set cost_scaling_factor parameter to 1.0 for inflation layer
  parameters.push_back(rclcpp::Parameter("inflation.cost_scaling_factor", 1.0));
  parameters.push_back(rclcpp::Parameter("inflation.inflation_radius", inflation_radius));

  initNode(parameters);
}

TEST_F(TestNode, testAdjacentToObstacleCanStillMove)
{
  initNode(4.1);
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  // Footprint with inscribed radius = 2.1
  //               circumscribed radius = 3.1
  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);

  layers.setFootprint(polygon);

  addObservation(olayer, 0, 0, MAX_Z);

  layers.updateMap(0, 0, 0);
  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  // printMap(*costmap);
  EXPECT_EQ(nav2_costmap_2d::LETHAL_OBSTACLE, costmap->getCost(0, 0));
  EXPECT_EQ(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, costmap->getCost(1, 0));
  EXPECT_EQ(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, costmap->getCost(2, 0));
  EXPECT_TRUE(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE > costmap->getCost(3, 0));
  EXPECT_TRUE(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE > costmap->getCost(2, 1));
  EXPECT_EQ(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, costmap->getCost(1, 1));
}

TEST_F(TestNode, testInflationShouldNotCreateUnknowns)
{
  initNode(4.1);
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  // Footprint with inscribed radius = 2.1
  // circumscribed radius = 3.1
  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);

  layers.setFootprint(polygon);

  addObservation(olayer, 0, 0, MAX_Z);

  layers.updateMap(0, 0, 0);
  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

  EXPECT_EQ(countValues(*costmap, nav2_costmap_2d::NO_INFORMATION), 0u);
}

TEST_F(TestNode, testInflationInUnknown)
{
  std::vector<rclcpp::Parameter> parameters;
  // Set cost_scaling_factor parameter to 1.0 for inflation layer
  parameters.push_back(rclcpp::Parameter("inflation.cost_scaling_factor", 1.0));
  parameters.push_back(rclcpp::Parameter("inflation.inflation_radius", 4.1));
  parameters.push_back(rclcpp::Parameter("inflation.inflate_unknown", true));

  initNode(parameters);

  node_->set_parameter(rclcpp::Parameter("track_unknown_space", true));

  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, true);
  layers.resizeMap(9, 9, 1, 0, 0);

  // Footprint with inscribed radius = 2.1
  // circumscribed radius = 3.1
  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);
  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);
  layers.setFootprint(polygon);

  addObservation(olayer, 4, 4, MAX_Z, 0.0, 0.0, MAX_Z, true, false);

  layers.updateMap(0, 0, 0);
  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

  // Only the 4 corners of the map should remain unknown
  EXPECT_EQ(countValues(*costmap, nav2_costmap_2d::NO_INFORMATION), 4u);
}

TEST_F(TestNode, testInflationAroundUnknown)
{
  auto inflation_radius = 4.1;
  std::vector<rclcpp::Parameter> parameters;
  // Set cost_scaling_factor parameter to 1.0 for inflation layer
  parameters.push_back(rclcpp::Parameter("inflation.cost_scaling_factor", 1.0));
  parameters.push_back(rclcpp::Parameter("inflation.inflation_radius", inflation_radius));
  parameters.push_back(rclcpp::Parameter("inflation.inflate_around_unknown", true));

  initNode(parameters);

  node_->set_parameter(rclcpp::Parameter("track_unknown_space", true));

  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  // Footprint with inscribed radius = 2.1
  // circumscribed radius = 3.1
  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);
  layers.setFootprint(polygon);
  layers.updateMap(0, 0, 0);

  layers.getCostmap()->setCost(4, 4, nav2_costmap_2d::NO_INFORMATION);
  ilayer->updateCosts(*layers.getCostmap(), 0, 0, 10, 10);

  validatePointInflation(4, 4, layers.getCostmap(), ilayer, inflation_radius);
}

/**
 * Test for the cost function correctness with a larger range and different values
 */
TEST_F(TestNode, testCostFunctionCorrectness)
{
  initNode(10.5);
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  layers.resizeMap(100, 100, 1, 0, 0);
  // Footprint with inscribed radius = 5.0
  //               circumscribed radius = 8.0
  std::vector<Point> polygon = setRadii(layers, 5.0, 6.25);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);

  layers.setFootprint(polygon);

  addObservation(olayer, 50, 50, MAX_Z);

  layers.updateMap(0, 0, 0);
  nav2_costmap_2d::Costmap2D * map = layers.getCostmap();

  // Verify that the circumscribed cost lower bound is as expected: based on the cost function.
  // unsigned char c = ilayer->computeCost(8.0);
  // ASSERT_EQ(ilayer->getCircumscribedCost(), c);

  for (unsigned int i = 0; i <= (unsigned int)ceil(5.0); i++) {
    // To the right
    ASSERT_EQ(map->getCost(50 + i, 50) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50 + i, 50) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // To the left
    ASSERT_EQ(map->getCost(50 - i, 50) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50 - i, 50) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // Down
    ASSERT_EQ(map->getCost(50, 50 + i) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50, 50 + i) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // Up
    ASSERT_EQ(map->getCost(50, 50 - i) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50, 50 - i) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
  }

  // Verify the normalized cost attenuates as expected
  for (unsigned int i = (unsigned int)(ceil(5.0) + 1); i <= (unsigned int)ceil(10.5); i++) {
    unsigned char expectedValue = ilayer->computeCost(i / 1.0);
    ASSERT_EQ(map->getCost(50 + i, 50), expectedValue);
  }

  // Update with no hits. Should clear (revert to the static map
  /*map->resetMapOutsideWindow(0, 0, 0.0, 0.0);
  cloud.points.resize(0);

  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs2(p, cloud, 100.0, 100.0);
  std::vector<Observation> obsBuf2;
  obsBuf2.push_back(obs2);

  map->updateWorld(0, 0, obsBuf2, obsBuf2);

  for(unsigned int i = 0; i < 100; i++)
    for(unsigned int j = 0; j < 100; j++)
      ASSERT_EQ(map->getCost(i, j), nav2_costmap_2d::FREE_SPACE);*/
}

/**
 * Test inflation around a large obstacle square on a large map
 * This tests the inflation layer's performance and correctness at scale
 */
TEST_F(TestNode, testLargeScaleInflation)
  {
    const double inflation_radius = 10.5;
    initNode(inflation_radius);
    tf2_ros::Buffer tf(node_->get_clock());
    nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

    // Create a 9000x9000 map
    layers.resizeMap(9000, 9000, 1, 0, 0);

    std::vector<Point> polygon = setRadii(layers, 5.0, 6.25);

    std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
    addObstacleLayer(layers, tf, node_, olayer);

    std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
    addInflationLayer(layers, tf, node_, ilayer);

    layers.setFootprint(polygon);

    // Create a 5000x5000 lethal square centered in the map
    // Square goes from (2000, 2000) to (6999, 6999)
    const unsigned int square_start = 2000;
    const unsigned int square_end = 6999;

    // First, update the map to initialize all layers
    layers.updateMap(0, 0, 0);
    nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

    // Directly set costs in the costmap for efficiency (instead of adding 25M observations)
    for (unsigned int x = square_start; x <= square_end; ++x) {
    for (unsigned int y = square_start; y <= square_end; ++y) {
      costmap->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
    }

    // Now run inflation on the updated costmap
    ilayer->updateCosts(*costmap, 0, 0, 9000, 9000);

    const int inflation_cells = static_cast<int>(std::ceil(inflation_radius /
      costmap->getResolution()));

    // Check all cells around the square perimeter within the inflation radius
    // This validates the inflation correctness comprehensively

    // Top edge: all cells above the square within inflation radius
    for (unsigned int x = square_start; x <= square_end; ++x) {
    for (int offset = 1; offset <= inflation_cells; ++offset) {
      const int test_y = static_cast<int>(square_start) - offset;
      if (test_y >= 0) {
        const unsigned char actual_cost = costmap->getCost(x, test_y);
        const unsigned char expected_cost = ilayer->computeCost(static_cast<double>(offset));
        ASSERT_EQ(actual_cost, expected_cost)
            << "Top edge: at (" << x << ", " << test_y << ") " << offset << " cells away";
      }
    }
    }

    // Bottom edge: all cells below the square within inflation radius
    for (unsigned int x = square_start; x <= square_end; ++x) {
    for (int offset = 1; offset <= inflation_cells; ++offset) {
      const unsigned int test_y = square_end + offset;
      if (test_y < costmap->getSizeInCellsY()) {
        const unsigned char actual_cost = costmap->getCost(x, test_y);
        const unsigned char expected_cost = ilayer->computeCost(static_cast<double>(offset));
        ASSERT_EQ(actual_cost, expected_cost)
            << "Bottom edge: at (" << x << ", " << test_y << ") " << offset << " cells away";
      }
    }
    }

    // Left edge: all cells to the left of the square within inflation radius
    for (unsigned int y = square_start; y <= square_end; ++y) {
    for (int offset = 1; offset <= inflation_cells; ++offset) {
      const int test_x = static_cast<int>(square_start) - offset;
      if (test_x >= 0) {
        const unsigned char actual_cost = costmap->getCost(test_x, y);
        const unsigned char expected_cost = ilayer->computeCost(static_cast<double>(offset));
        ASSERT_EQ(actual_cost, expected_cost)
            << "Left edge: at (" << test_x << ", " << y << ") " << offset << " cells away";
      }
    }
    }

    // Right edge: all cells to the right of the square within inflation radius
    for (unsigned int y = square_start; y <= square_end; ++y) {
    for (int offset = 1; offset <= inflation_cells; ++offset) {
      const unsigned int test_x = square_end + offset;
      if (test_x < costmap->getSizeInCellsX()) {
        const unsigned char actual_cost = costmap->getCost(test_x, y);
        const unsigned char expected_cost = ilayer->computeCost(static_cast<double>(offset));
        ASSERT_EQ(actual_cost, expected_cost)
            << "Right edge: at (" << test_x << ", " << y << ") " << offset << " cells away";
      }
    }
    }

    // Additionally, verify that cells just outside the inflation radius are free space
    // Check a point far from any obstacle
    const unsigned int far_x = 100;
    const unsigned int far_y = 100;
    ASSERT_EQ(costmap->getCost(far_x, far_y), nav2_costmap_2d::FREE_SPACE)
      << "Cell far from obstacles should be FREE_SPACE";

    // Verify cells just outside the square but within inflation radius have appropriate costs
    const int test_x_signed = static_cast<int>(square_start) - inflation_cells / 2;
    const int test_y_signed = static_cast<int>(square_start) - inflation_cells / 2;
    if (test_x_signed >= 0 && test_y_signed >= 0) {
    const unsigned int test_x = static_cast<unsigned int>(test_x_signed);
    const unsigned int test_y = static_cast<unsigned int>(test_y_signed);
    const unsigned char cost = costmap->getCost(test_x, test_y);
    ASSERT_GT(cost, nav2_costmap_2d::FREE_SPACE)
        << "Cell within inflation radius should have inflated cost";
    }
}

/**
 * Test that there is no regression and that costs do not get
 * underestimated with the distance-as-key map used to replace
 * the previously used priority queue. This is a more thorough
 * test of the cost function being correctly applied.
 */
TEST_F(TestNode, testInflationOrderCorrectness)
{
  const double inflation_radius = 4.1;
  initNode(inflation_radius);
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  // Footprint with inscribed radius = 2.1
  //               circumscribed radius = 3.1
  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);

  layers.setFootprint(polygon);

  // Add two diagonal cells, they would induce problems under the
  // previous implementations
  addObservation(olayer, 4, 4, MAX_Z);
  addObservation(olayer, 5, 5, MAX_Z);

  layers.updateMap(0, 0, 0);

  validatePointInflation(4, 4, layers.getCostmap(), ilayer, inflation_radius);
  validatePointInflation(5, 5, layers.getCostmap(), ilayer, inflation_radius);
}

/**
 * Test inflation for both static and dynamic obstacles
 */
TEST_F(TestNode, testInflation)
{
  initNode(1);
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  // Footprint with inscribed radius = 2.1
  // circumscribed radius = 3.1
  std::vector<Point> polygon = setRadii(layers, 1, 1);

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer = nullptr;
  addStaticLayer(layers, tf, node_, slayer);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);
  layers.setFootprint(polygon);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  waitForMap(slayer);

  layers.updateMap(0, 0, 0);
  // printMap(*costmap);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 20u);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 28u);

  /*/ Iterate over all id's and verify they are obstacles
  for(std::vector<unsigned int>::const_iterator it = occupiedCells.begin(); it != occupiedCells.end(); ++it){
    unsigned int ind = *it;
    unsigned int x, y;
    map.indexToCells(ind, x, y);
    ASSERT_EQ(find(occupiedCells, map.getIndex(x, y)), true);
    ASSERT_EQ(map.getCost(x, y) == nav2_costmap_2d::LETHAL_OBSTACLE ||
      map.getCost(x, y) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
  }*/

  addObservation(olayer, 0, 0, 0.4);
  layers.updateMap(0, 0, 0);

  // It and its 2 neighbors makes 3 obstacles
  ASSERT_EQ(
    countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE) +
    countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 51u);

  // @todo Rewrite
  // Add an obstacle at <2,0> which will inflate and refresh to of the other inflated cells
  addObservation(olayer, 2, 0);
  layers.updateMap(0, 0, 0);

  // Now we expect insertions for it, and 2 more neighbors, but not all 5.
  // Free space will propagate from
  // the origin to the target, clearing the point at <0, 0>,
  // but not over-writing the inflation of the obstacle
  // at <0, 1>
  ASSERT_EQ(
    countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE) +
    countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 54u);

  // Add an obstacle at <1, 9>. This will inflate obstacles around it
  addObservation(olayer, 1, 9);
  layers.updateMap(0, 0, 0);

  ASSERT_EQ(costmap->getCost(1, 9), nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_EQ(costmap->getCost(0, 9), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  ASSERT_EQ(costmap->getCost(2, 9), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  // Add an obstacle and verify that it over-writes its inflated status
  addObservation(olayer, 0, 9);
  layers.updateMap(0, 0, 0);

  ASSERT_EQ(costmap->getCost(0, 9), nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test specific inflation scenario to ensure we do not set inflated obstacles to be raw obstacles.
 */
TEST_F(TestNode, testInflation2)
{
  initNode(1);
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  // Footprint with inscribed radius = 2.1
  // circumscribed radius = 3.1
  std::vector<Point> polygon = setRadii(layers, 1, 1);

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer = nullptr;
  addStaticLayer(layers, tf, node_, slayer);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);

  layers.setFootprint(polygon);

  waitForMap(slayer);

  // Create a small L-Shape all at once
  addObservation(olayer, 1, 1, MAX_Z);
  addObservation(olayer, 2, 1, MAX_Z);
  addObservation(olayer, 2, 2, MAX_Z);
  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  // printMap(*costmap);
  ASSERT_EQ(costmap->getCost(2, 3), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  ASSERT_EQ(costmap->getCost(3, 3), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

/**
 * Test inflation behavior, starting with an empty map
 */
TEST_F(TestNode, testInflation3)
{
  initNode(3);
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  // 1 2 3
  std::vector<Point> polygon = setRadii(layers, 1, 1.75);

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer = nullptr;
  addObstacleLayer(layers, tf, node_, olayer);

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer = nullptr;
  addInflationLayer(layers, tf, node_, ilayer);

  layers.setFootprint(polygon);

  // There should be no occupied cells
  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 0u);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 0u);
  printMap(*costmap);
  // Add an obstacle at 5,5
  addObservation(olayer, 5, 5, MAX_Z);
  layers.updateMap(0, 0, 0);
  printMap(*costmap);

  // Test fails because updated cell value is 0
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::FREE_SPACE, false), 29u);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 1u);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 4u);

  // Update again - should see no change
  layers.updateMap(0, 0, 0);

  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::FREE_SPACE, false), 29u);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 1u);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 4u);
}

/**
 * Test dynamic parameter setting of inflation layer
 */
TEST_F(TestNode, testDynParamsSet)
{
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  costmap->set_parameter(rclcpp::Parameter("global_frame", std::string("base_link")));
  costmap->on_configure(rclcpp_lifecycle::State());

  costmap->on_activate(rclcpp_lifecycle::State());

  auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(
    costmap->get_node_base_interface(), costmap->get_node_topics_interface(),
    costmap->get_node_graph_interface(),
    costmap->get_node_services_interface());

  auto results = parameter_client->set_parameters_atomically(
  {
    rclcpp::Parameter("inflation_layer.inflation_radius", 0.0),
    rclcpp::Parameter("inflation_layer.cost_scaling_factor", 0.0),
    rclcpp::Parameter("inflation_layer.inflate_unknown", true),
    rclcpp::Parameter("inflation_layer.inflate_around_unknown", true),
    rclcpp::Parameter("inflation_layer.enabled", false)
  });

  rclcpp::spin_until_future_complete(
    costmap->get_node_base_interface(),
    results);

  EXPECT_EQ(costmap->get_parameter("inflation_layer.inflation_radius").as_double(), 0.0);
  EXPECT_EQ(costmap->get_parameter("inflation_layer.cost_scaling_factor").as_double(), 0.0);
  EXPECT_EQ(costmap->get_parameter("inflation_layer.inflate_unknown").as_bool(), true);
  EXPECT_EQ(costmap->get_parameter("inflation_layer.inflate_around_unknown").as_bool(), true);
  EXPECT_EQ(costmap->get_parameter("inflation_layer.enabled").as_bool(), false);

  costmap->on_deactivate(rclcpp_lifecycle::State());
  costmap->on_cleanup(rclcpp_lifecycle::State());
  costmap->on_shutdown(rclcpp_lifecycle::State());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
