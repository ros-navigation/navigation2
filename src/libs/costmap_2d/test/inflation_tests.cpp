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
#include <map>
#include <cmath>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/testing_helper.h>
#include <gtest/gtest.h>

using namespace costmap_2d;
using geometry_msgs::Point;

std::vector<Point> setRadii(LayeredCostmap& layers, double length, double width, double inflation_radius)
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

  ros::NodeHandle nh;
  nh.setParam("/inflation_tests/inflation/inflation_radius", inflation_radius);

  return polygon;
}

// Test that a single point gets inflated properly
void validatePointInflation(unsigned int mx, unsigned int my, Costmap2D* costmap, InflationLayer* ilayer, double inflation_radius)
{
  bool* seen = new bool[costmap->getSizeInCellsX() * costmap->getSizeInCellsY()];
  memset(seen, false, costmap->getSizeInCellsX() * costmap->getSizeInCellsY() * sizeof(bool));
  std::map<double, std::vector<CellData> > m;
  CellData initial(costmap->getIndex(mx, my), mx, my, mx, my);
  m[0].push_back(initial);
  for (std::map<double, std::vector<CellData> >::iterator bin = m.begin(); bin != m.end(); ++bin)
  {
    for (int i = 0; i < bin->second.size(); ++i)
    {
      const CellData& cell = bin->second[i];
      if (!seen[cell.index_])
      {
        seen[cell.index_] = true;
        unsigned int dx = (cell.x_ > cell.src_x_) ? cell.x_ - cell.src_x_ : cell.src_x_ - cell.x_;
        unsigned int dy = (cell.y_ > cell.src_y_) ? cell.y_ - cell.src_y_ : cell.src_y_ - cell.y_;
        double dist = hypot(dx, dy);

        unsigned char expected_cost = ilayer->computeCost(dist);
        ASSERT_TRUE(costmap->getCost(cell.x_, cell.y_) >= expected_cost);

        if (dist > inflation_radius)
        {
          continue;
        }

        if (cell.x_ > 0)
        {
          CellData data(costmap->getIndex(cell.x_-1, cell.y_),
                        cell.x_-1, cell.y_, cell.src_x_, cell.src_y_);
          m[dist].push_back(data);
        }
        if (cell.y_ > 0)
        {
          CellData data(costmap->getIndex(cell.x_, cell.y_-1),
                        cell.x_, cell.y_-1, cell.src_x_, cell.src_y_);
          m[dist].push_back(data);
        }
        if (cell.x_ < costmap->getSizeInCellsX() - 1)
        {
          CellData data(costmap->getIndex(cell.x_+1, cell.y_),
                        cell.x_+1, cell.y_, cell.src_x_, cell.src_y_);
          m[dist].push_back(data);
        }
        if (cell.y_ < costmap->getSizeInCellsY() - 1)
        {
          CellData data(costmap->getIndex(cell.x_, cell.y_+1),
                        cell.x_, cell.y_+1, cell.src_x_, cell.src_y_);
          m[dist].push_back(data);
        }
      }
    }
  }
  delete[] seen;
}

TEST(costmap, testAdjacentToObstacleCanStillMove){
  tf2_ros::Buffer tf;
  LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  // Footprint with inscribed radius = 2.1
  //               circumscribed radius = 3.1
  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3, 4.1);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);  
  InflationLayer* ilayer = addInflationLayer(layers, tf);
  layers.setFootprint(polygon);

  addObservation(olayer, 0, 0, MAX_Z);

  layers.updateMap(0,0,0);
  Costmap2D* costmap = layers.getCostmap();
  //printMap(*costmap);
  EXPECT_EQ( LETHAL_OBSTACLE, costmap->getCost( 0, 0 ));
  EXPECT_EQ( INSCRIBED_INFLATED_OBSTACLE, costmap->getCost( 1, 0 ));
  EXPECT_EQ( INSCRIBED_INFLATED_OBSTACLE, costmap->getCost( 2, 0 ));
  EXPECT_TRUE( INSCRIBED_INFLATED_OBSTACLE > costmap->getCost( 3, 0 ));
  EXPECT_TRUE( INSCRIBED_INFLATED_OBSTACLE > costmap->getCost( 2, 1 ));
  EXPECT_EQ( INSCRIBED_INFLATED_OBSTACLE, costmap->getCost( 1, 1 ));
}

TEST(costmap, testInflationShouldNotCreateUnknowns){
  tf2_ros::Buffer tf;
  LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  // Footprint with inscribed radius = 2.1
  //               circumscribed radius = 3.1
  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3, 4.1);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);  
  InflationLayer* ilayer = addInflationLayer(layers, tf);
  layers.setFootprint(polygon);

  addObservation(olayer, 0, 0, MAX_Z);

  layers.updateMap(0,0,0);
  Costmap2D* costmap = layers.getCostmap();

  EXPECT_EQ( countValues(*costmap, NO_INFORMATION), 0 );
}


/**
 * Test for the cost function correctness with a larger range and different values
 */
TEST(costmap, testCostFunctionCorrectness){
  tf2_ros::Buffer tf;
  LayeredCostmap layers("frame", false, false);
  layers.resizeMap(100, 100, 1, 0, 0);

  // Footprint with inscribed radius = 5.0
  //               circumscribed radius = 8.0
  std::vector<Point> polygon = setRadii(layers, 5.0, 6.25, 10.5);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);  
  InflationLayer* ilayer = addInflationLayer(layers, tf);
  layers.setFootprint(polygon);

  addObservation(olayer, 50, 50, MAX_Z);

  layers.updateMap(0,0,0);
  Costmap2D* map = layers.getCostmap();

  // Verify that the circumscribed cost lower bound is as expected: based on the cost function.
  //unsigned char c = ilayer->computeCost(8.0);
  //ASSERT_EQ(ilayer->getCircumscribedCost(), c);

  for(unsigned int i = 0; i <= (unsigned int)ceil(5.0); i++){
    // To the right
    ASSERT_EQ(map->getCost(50 + i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50 + i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // To the left
    ASSERT_EQ(map->getCost(50 - i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50 - i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // Down
    ASSERT_EQ(map->getCost(50, 50 + i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50, 50 + i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // Up
    ASSERT_EQ(map->getCost(50, 50 - i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50, 50 - i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
  }

  // Verify the normalized cost attenuates as expected
  for(unsigned int i = (unsigned int)(ceil(5.0) + 1); i <= (unsigned int)ceil(10.5); i++){
    unsigned char expectedValue = ilayer->computeCost(i/1.0);
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
      ASSERT_EQ(map->getCost(i, j), costmap_2d::FREE_SPACE);*/
}

/**
 * Test that there is no regression and that costs do not get
 * underestimated with the distance-as-key map used to replace
 * the previously used priority queue. This is a more thorough
 * test of the cost function being correctly applied.
 */
TEST(costmap, testInflationOrderCorrectness){
  tf2_ros::Buffer tf;
  LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  // Footprint with inscribed radius = 2.1
  //               circumscribed radius = 3.1
  const double inflation_radius = 4.1;
  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3, inflation_radius);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);
  InflationLayer* ilayer = addInflationLayer(layers, tf);
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
TEST(costmap, testInflation){

  tf2_ros::Buffer tf;
  LayeredCostmap layers("frame", false, false);

  // Footprint with inscribed radius = 2.1
  //               circumscribed radius = 3.1
  std::vector<Point> polygon = setRadii(layers, 1, 1, 1);

  addStaticLayer(layers, tf);
  ObstacleLayer* olayer = addObstacleLayer(layers, tf);  
  InflationLayer* ilayer = addInflationLayer(layers, tf);
  layers.setFootprint(polygon);

  Costmap2D* costmap = layers.getCostmap();

  layers.updateMap(0,0,0);
  //printMap(*costmap);
  ASSERT_EQ(countValues(*costmap, LETHAL_OBSTACLE),             (unsigned int)20);
  ASSERT_EQ(countValues(*costmap, INSCRIBED_INFLATED_OBSTACLE), (unsigned int)28);

  /*/ Iterate over all id's and verify they are obstacles
  for(std::vector<unsigned int>::const_iterator it = occupiedCells.begin(); it != occupiedCells.end(); ++it){
    unsigned int ind = *it;
    unsigned int x, y;
    map.indexToCells(ind, x, y);
    ASSERT_EQ(find(occupiedCells, map.getIndex(x, y)), true);
    ASSERT_EQ(map.getCost(x, y) == costmap_2d::LETHAL_OBSTACLE || map.getCost(x, y) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
  }*/

  addObservation(olayer, 0, 0, 0.4);
  layers.updateMap(0,0,0);

  // It and its 2 neighbors makes 3 obstacles
  ASSERT_EQ(countValues(*costmap, LETHAL_OBSTACLE) + countValues(*costmap, INSCRIBED_INFLATED_OBSTACLE), (unsigned int)51);

  // @todo Rewrite 
  // Add an obstacle at <2,0> which will inflate and refresh to of the other inflated cells
  addObservation(olayer, 2, 0);
  layers.updateMap(0,0,0);

  // Now we expect insertions for it, and 2 more neighbors, but not all 5. Free space will propagate from
  // the origin to the target, clearing the point at <0, 0>, but not over-writing the inflation of the obstacle
  // at <0, 1>
  ASSERT_EQ(countValues(*costmap, LETHAL_OBSTACLE) + countValues(*costmap, INSCRIBED_INFLATED_OBSTACLE), (unsigned int)54);

  // Add an obstacle at <1, 9>. This will inflate obstacles around it
  addObservation(olayer, 1, 9);
  layers.updateMap(0,0,0);

  ASSERT_EQ(costmap->getCost(1, 9), LETHAL_OBSTACLE);
  ASSERT_EQ(costmap->getCost(0, 9), INSCRIBED_INFLATED_OBSTACLE);
  ASSERT_EQ(costmap->getCost(2, 9), INSCRIBED_INFLATED_OBSTACLE);

  // Add an obstacle and verify that it over-writes its inflated status
  addObservation(olayer, 0, 9);
  layers.updateMap(0,0,0);

  ASSERT_EQ(costmap->getCost(0, 9), LETHAL_OBSTACLE);
}

/**
 * Test specific inflation scenario to ensure we do not set inflated obstacles to be raw obstacles.
 */
TEST(costmap, testInflation2){

  tf2_ros::Buffer tf;
  LayeredCostmap layers("frame", false, false);

  // Footprint with inscribed radius = 2.1
  //               circumscribed radius = 3.1
  std::vector<Point> polygon = setRadii(layers, 1, 1, 1);

  addStaticLayer(layers, tf);
  ObstacleLayer* olayer = addObstacleLayer(layers, tf);  
  InflationLayer* ilayer = addInflationLayer(layers, tf);
  layers.setFootprint(polygon);

  // Creat a small L-Shape all at once
  addObservation(olayer, 1, 1, MAX_Z);
  addObservation(olayer, 2, 1, MAX_Z);
  addObservation(olayer, 2, 2, MAX_Z);
  layers.updateMap(0,0,0);

  Costmap2D* costmap = layers.getCostmap();
  //printMap(*costmap);
  ASSERT_EQ(costmap->getCost(2, 3), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);  
  ASSERT_EQ(costmap->getCost(3, 3), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

/**
 * Test inflation behavior, starting with an empty map
 */
TEST(costmap, testInflation3){
  tf2_ros::Buffer tf;
  LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  // 1 2 3
  std::vector<Point> polygon = setRadii(layers, 1, 1.75, 3);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);  
  InflationLayer* ilayer = addInflationLayer(layers, tf);
  layers.setFootprint(polygon);

  // There should be no occupied cells
  Costmap2D* costmap = layers.getCostmap();
  ASSERT_EQ(countValues(*costmap, LETHAL_OBSTACLE),             (unsigned int)0);
  ASSERT_EQ(countValues(*costmap, INSCRIBED_INFLATED_OBSTACLE), (unsigned int)0);
  printMap(*costmap);
  // Add an obstacle at 5,5
  addObservation(olayer, 5, 5, MAX_Z);
  layers.updateMap(0,0,0);
  printMap(*costmap);

  // Test fails because updated cell value is 0
  ASSERT_EQ(countValues(*costmap, FREE_SPACE, false), (unsigned int)29);
  ASSERT_EQ(countValues(*costmap, LETHAL_OBSTACLE), (unsigned int)1);
  ASSERT_EQ(countValues(*costmap, INSCRIBED_INFLATED_OBSTACLE), (unsigned int)4);

  // Update again - should see no change
  layers.updateMap(0,0,0);

  ASSERT_EQ(countValues(*costmap, FREE_SPACE, false), (unsigned int)29);
  ASSERT_EQ(countValues(*costmap, LETHAL_OBSTACLE), (unsigned int)1);
  ASSERT_EQ(countValues(*costmap, INSCRIBED_INFLATED_OBSTACLE), (unsigned int)4);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "inflation_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
