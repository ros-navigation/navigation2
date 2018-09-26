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
 * Test harness for ObstacleLayer for Costmap2D
 */

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/testing_helper.h>
#include <set>
#include <gtest/gtest.h>
#include <tf/transform_listener.h>

using namespace costmap_2d;

/*
 * For reference, the static map looks like this:
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0 254 254 254   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0 254   0   0 254 254 254
 *
 *   0   0   0   0 254   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   upper left is 0,0, lower right is 9,9
 */

/**
 * Test for ray tracing free space
 */
TEST(costmap, testRaytracing){
  tf::TransformListener tf;

  LayeredCostmap layers("frame", false, false);  // Not rolling window, not tracking unknown
  addStaticLayer(layers, tf);  // This adds the static map
  ObstacleLayer* olayer = addObstacleLayer(layers, tf);
  
  // Add a point at 0, 0, 0
  addObservation(olayer, 0.0, 0.0, MAX_Z/2, 0, 0, MAX_Z/2);

  // This actually puts the LETHAL (254) point in the costmap at (0,0)
  layers.updateMap(0,0,0);  // 0, 0, 0 is robot pose
  //printMap(*(layers.getCostmap()));
  
  int lethal_count = countValues(*(layers.getCostmap()), LETHAL_OBSTACLE);

  // We expect just one obstacle to be added (20 in static map)
  ASSERT_EQ(lethal_count, 21);
}

/**
 * Test for ray tracing free space
 */
TEST(costmap, testRaytracing2){
  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf);
  ObstacleLayer* olayer = addObstacleLayer(layers, tf);

  //If we print map now, it is 10x10 all value 0
  //printMap(*(layers.getCostmap()));

  // Update will fill in the costmap with the static map
  layers.updateMap(0,0,0);

  //If we print the map now, we get the static map
  //printMap(*(layers.getCostmap()));

  // Static map has 20 LETHAL cells (see diagram above)
  int obs_before = countValues(*(layers.getCostmap()), LETHAL_OBSTACLE);
  ASSERT_EQ(obs_before, 20);

  // The sensor origin will be <0,0>. So if we add an obstacle at 9,9,
  // we would expect cells <0, 0> thru <8, 8> to be traced through
  // however the static map is not cleared by obstacle layer
  addObservation(olayer, 9.5, 9.5, MAX_Z/2, 0.5, 0.5, MAX_Z/2);
  layers.updateMap(0, 0, 0);

  // If we print map now, we have static map + <9,9> is LETHAL
  //printMap(*(layers.getCostmap()));
  int obs_after = countValues(*(layers.getCostmap()), LETHAL_OBSTACLE);

  // Change from previous test:
  // No obstacles from the static map will be cleared, so the
  // net change is +1. 
  ASSERT_EQ(obs_after, obs_before + 1);

  // Fill in the diagonal, <7,7> and <9,9> already filled in, <0,0> is robot
  for(int i = 0; i < olayer->getSizeInCellsY(); ++i)
  {
    olayer->setCost(i, i, LETHAL_OBSTACLE);
  }
  // This will updateBounds, which will raytrace the static observation added
  // above, thus clearing out the diagonal again!
  layers.updateMap(0, 0, 0);

  // Map now has diagonal except <0,0> filled with LETHAL (254)
  //printMap(*(layers.getCostmap()));
  int with_static = countValues(*(layers.getCostmap()), LETHAL_OBSTACLE);

  // Should thus be the same
  ASSERT_EQ(with_static, obs_after);
  // If 21 are filled, 79 should be free
  ASSERT_EQ(79, countValues(*(layers.getCostmap()), costmap_2d::FREE_SPACE));
}

/**
 * Test for wave interference
 */
TEST(costmap, testWaveInterference){
  tf::TransformListener tf;

  // Start with an empty map, no rolling window, tracking unknown
  LayeredCostmap layers("frame", false, true);
  layers.resizeMap(10, 10, 1, 0, 0);
  ObstacleLayer* olayer = addObstacleLayer(layers, tf);

  // If we print map now, it is 10x10, all cells are 255 (NO_INFORMATION)
  //printMap(*(layers.getCostmap()));

  // Lay out 3 obstacles in a line - along the diagonal, separated by a cell.
  addObservation(olayer, 3.0, 3.0, MAX_Z);
  addObservation(olayer, 5.0, 5.0, MAX_Z);
  addObservation(olayer, 7.0, 7.0, MAX_Z);
  layers.updateMap(0,0,0);

  Costmap2D* costmap = layers.getCostmap();
  // 3 obstacle cells are filled, <1,1>,<2,2>,<4,4> and <6,6> are now free
  // <0,0> is footprint and is free
  //printMap(*costmap);
  ASSERT_EQ(3, countValues(*costmap, costmap_2d::LETHAL_OBSTACLE));
  ASSERT_EQ(92, countValues(*costmap, costmap_2d::NO_INFORMATION));
  ASSERT_EQ(5, countValues(*costmap, costmap_2d::FREE_SPACE));
}

/**
 * Make sure we ignore points outside of our z threshold
 */
TEST(costmap, testZThreshold){
  tf::TransformListener tf;
  // Start with an empty map
  LayeredCostmap layers("frame", false, true);
  layers.resizeMap(10, 10, 1, 0, 0);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);

  // A point cloud with 2 points falling in a cell with a non-lethal cost
  addObservation(olayer, 0.0, 5.0, 0.4);
  addObservation(olayer, 1.0, 5.0, 2.2);

  layers.updateMap(0,0,0);

  Costmap2D* costmap = layers.getCostmap();
  ASSERT_EQ(countValues(*costmap, costmap_2d::LETHAL_OBSTACLE), 1);
}


/**
 * Verify that dynamic obstacles are added
 */
TEST(costmap, testDynamicObstacles){
  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);

  // Add a point cloud and verify its insertion. There should be only one new one
  addObservation(olayer, 0.0, 0.0);
  addObservation(olayer, 0.0, 0.0);
  addObservation(olayer, 0.0, 0.0);

  layers.updateMap(0,0,0);

  Costmap2D* costmap = layers.getCostmap();
  // Should now have 1 insertion and no deletions
  ASSERT_EQ(countValues(*costmap, costmap_2d::LETHAL_OBSTACLE), 21);

  // Repeating the call - we should see no insertions or deletions
  ASSERT_EQ(countValues(*costmap, costmap_2d::LETHAL_OBSTACLE), 21);
}


/**
 * Verify that if we add a point that is already a static obstacle we do not end up with a new ostacle
 */
TEST(costmap, testMultipleAdditions){
  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);

  // A point cloud with one point that falls within an existing obstacle
  addObservation(olayer, 9.5, 0.0);
  layers.updateMap(0,0,0);
  Costmap2D* costmap = layers.getCostmap();
  //printMap(*costmap);

  ASSERT_EQ(countValues(*costmap, costmap_2d::LETHAL_OBSTACLE), 20);

}


int main(int argc, char** argv){
  ros::init(argc, argv, "obstacle_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
