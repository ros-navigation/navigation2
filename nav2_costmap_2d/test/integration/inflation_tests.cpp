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
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

using geometry_msgs::msg::Point;
using nav2_costmap_2d::CellData;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

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
  nav2_util::LifecycleNode::SharedPtr node_;
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
  while (!slayer->isCurrent()) {
    rclcpp::spin_some(node_->get_node_base_interface());
  }
}

// Test that a single point gets inflated properly
void TestNode::validatePointInflation(
  unsigned int mx, unsigned int my,
  nav2_costmap_2d::Costmap2D * costmap,
  std::shared_ptr<nav2_costmap_2d::InflationLayer> & ilayer,
  double inflation_radius)
{
  bool * seen = new bool[costmap->getSizeInCellsX() * costmap->getSizeInCellsY()];
  memset(seen, false, costmap->getSizeInCellsX() * costmap->getSizeInCellsY() * sizeof(bool));
  std::map<double, std::vector<CellData>> m;
  CellData initial(mx, my, mx, my);
  m[0].push_back(initial);
  for (std::map<double, std::vector<CellData>>::iterator bin = m.begin();
    bin != m.end(); ++bin)
  {
    for (unsigned int i = 0; i < bin->second.size(); ++i) {
      const CellData cell = bin->second[i];
      const auto index = costmap->getIndex(cell.x_, cell.y_);
      if (!seen[index]) {
        seen[index] = true;
        unsigned int dx = (cell.x_ > cell.src_x_) ? cell.x_ - cell.src_x_ : cell.src_x_ - cell.x_;
        unsigned int dy = (cell.y_ > cell.src_y_) ? cell.y_ - cell.src_y_ : cell.src_y_ - cell.y_;
        double dist = std::hypot(dx, dy);

        unsigned char expected_cost = ilayer->computeCost(dist);
        ASSERT_TRUE(costmap->getCost(cell.x_, cell.y_) >= expected_cost);

        if (dist > inflation_radius) {
          continue;
        }

        if (dist == bin->first) {
          // Adding to our current bin could cause a reallocation
          // Which appears to cause the iterator to get messed up
          dist += 0.001;
        }

        if (cell.x_ > 0) {
          CellData data(cell.x_ - 1, cell.y_, cell.src_x_, cell.src_y_);
          m[dist].push_back(data);
        }
        if (cell.y_ > 0) {
          CellData data(cell.x_, cell.y_ - 1, cell.src_x_, cell.src_y_);
          m[dist].push_back(data);
        }
        if (cell.x_ < costmap->getSizeInCellsX() - 1) {
          CellData data(cell.x_ + 1, cell.y_, cell.src_x_, cell.src_y_);
          m[dist].push_back(data);
        }
        if (cell.y_ < costmap->getSizeInCellsY() - 1) {
          CellData data(cell.x_, cell.y_ + 1, cell.src_x_, cell.src_y_);
          m[dist].push_back(data);
        }
      }
    }
  }
  delete[] seen;
}

void TestNode::initNode(std::vector<rclcpp::Parameter> parameters)
{
  auto options = rclcpp::NodeOptions();
  options.parameter_overrides(parameters);

  node_ = std::make_shared<nav2_util::LifecycleNode>(
    "inflation_test_node", "", options);

  // Declare non-plugin specific costmap parameters
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

TEST_F(TestNode, testInflationInUnkown)
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

TEST_F(TestNode, testInflationAroundUnkown)
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

  // Creat a small L-Shape all at once
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
