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
 * Test harness for StaticMap Layer for Costmap2D
 */

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/static_layer.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/testing_helper.h>
#include <set>
#include <gtest/gtest.h>

using namespace costmap_2d;


/**
 * Tests the reset method
 *
TEST(costmap, testResetForStaticMap){
  // Define a static map with a large object in the center
  std::vector<unsigned char> staticMap;
  for(unsigned int i=0; i<10; i++){
    for(unsigned int j=0; j<10; j++){
      staticMap.push_back(costmap_2d::LETHAL_OBSTACLE);
    }
  }

  // Allocate the cost map, with a inflation to 3 cells all around
  Costmap2D map(10, 10, RESOLUTION, 0.0, 0.0, 3, 3, 3, OBSTACLE_RANGE, MAX_Z, RAYTRACE_RANGE, 25, staticMap, THRESHOLD);

  // Populate the cost map with a wall around the perimeter. Free space should clear out the room.
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(40);

  // Left wall
  unsigned int ind = 0;
  for (unsigned int i = 0; i < 10; i++){
    // Left
    cloud.points[ind].x = 0;
    cloud.points[ind].y = i;
    cloud.points[ind].z = MAX_Z;
    ind++;

    // Top
    cloud.points[ind].x = i;
    cloud.points[ind].y = 0;
    cloud.points[ind].z = MAX_Z;
    ind++;

    // Right
    cloud.points[ind].x = 9;
    cloud.points[ind].y = i;
    cloud.points[ind].z = MAX_Z;
    ind++;

    // Bottom
    cloud.points[ind].x = i;
    cloud.points[ind].y = 9;
    cloud.points[ind].z = MAX_Z;
    ind++;
  }

  double wx = 5.0, wy = 5.0;
  geometry_msgs::Point p;
  p.x = wx;
  p.y = wy;
  p.z = MAX_Z;
  Observation obs(p, cloud, OBSTACLE_RANGE, RAYTRACE_RANGE);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  // Update the cost map for this observation
  map.updateWorld(wx, wy, obsBuf, obsBuf);

  // Verify that we now have only 36 cells with lethal cost, thus provong that we have correctly cleared
  // free space
  int hitCount = 0;
  for(unsigned int i=0; i < 10; ++i){
    for(unsigned int j=0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE){
        hitCount++;
      }
    }
  }
  ASSERT_EQ(hitCount, 36);

  // Veriy that we have 64 non-leathal
  hitCount = 0;
  for(unsigned int i=0; i < 10; ++i){
    for(unsigned int j=0; j < 10; ++j){
      if(map.getCost(i, j) != costmap_2d::LETHAL_OBSTACLE)
        hitCount++;
    }
  }
  ASSERT_EQ(hitCount, 64);

  // Now if we reset the cost map, we should have our map go back to being completely occupied
  map.resetMapOutsideWindow(wx, wy, 0.0, 0.0);

  //We should now go back to everything being occupied
  hitCount = 0;
  for(unsigned int i=0; i < 10; ++i){
    for(unsigned int j=0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
        hitCount++;
    }
  }
  ASSERT_EQ(hitCount, 100);

}

/** Test for copying a window of a costmap *
TEST(costmap, testWindowCopy){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
      10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  /*
  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      printf("%3d ", map.getCost(i, j));
    }
    printf("\n");
  }
  printf("\n");
  *

  Costmap2D windowCopy;

  //first test that if we try to make a window that is too big, things fail
  windowCopy.copyCostmapWindow(map, 2.0, 2.0, 6.0, 12.0);
  ASSERT_EQ(windowCopy.getSizeInCellsX(), (unsigned int)0);
  ASSERT_EQ(windowCopy.getSizeInCellsY(), (unsigned int)0);

  //Next, test that trying to make a map window itself fails
  map.copyCostmapWindow(map, 2.0, 2.0, 6.0, 6.0);
  ASSERT_EQ(map.getSizeInCellsX(), (unsigned int)10);
  ASSERT_EQ(map.getSizeInCellsY(), (unsigned int)10);

  //Next, test that legal input generates the result that we expect
  windowCopy.copyCostmapWindow(map, 2.0, 2.0, 6.0, 6.0);
  ASSERT_EQ(windowCopy.getSizeInCellsX(), (unsigned int)6);
  ASSERT_EQ(windowCopy.getSizeInCellsY(), (unsigned int)6);

  //check that we actually get the windo that we expect
  for(unsigned int i = 0; i < windowCopy.getSizeInCellsX(); ++i){
    for(unsigned int j = 0; j < windowCopy.getSizeInCellsY(); ++j){
      ASSERT_EQ(windowCopy.getCost(i, j), map.getCost(i + 2, j + 2));
      //printf("%3d ", windowCopy.getCost(i, j));
    }
    //printf("\n");
  }

}

//test for updating costmaps with static data
TEST(costmap, testFullyContainedStaticMapUpdate){
  Costmap2D map(5, 5, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
      10.0, MAX_Z, 10.0, 25, MAP_5_BY_5, THRESHOLD);

  Costmap2D static_map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
      10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  map.updateStaticMapWindow(0, 0, 10, 10, MAP_10_BY_10);

  for(unsigned int i = 0; i < map.getSizeInCellsX(); ++i){
    for(unsigned int j = 0; j < map.getSizeInCellsY(); ++j){
      ASSERT_EQ(map.getCost(i, j), static_map.getCost(i, j));
    }
  }
}

TEST(costmap, testOverlapStaticMapUpdate){
  Costmap2D map(5, 5, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
      10.0, MAX_Z, 10.0, 25, MAP_5_BY_5, THRESHOLD);

  Costmap2D static_map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
      10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  map.updateStaticMapWindow(-10, -10, 10, 10, MAP_10_BY_10);

  ASSERT_FLOAT_EQ(map.getOriginX(), -10);
  ASSERT_FLOAT_EQ(map.getOriginX(), -10);
  ASSERT_EQ(map.getSizeInCellsX(), (unsigned int)15);
  ASSERT_EQ(map.getSizeInCellsY(), (unsigned int)15);
  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      ASSERT_EQ(map.getCost(i, j), static_map.getCost(i, j));
    }
  }

  std::vector<unsigned char> blank(100);

  //check to make sure that inflation on updates are being done correctly
  map.updateStaticMapWindow(-10, -10, 10, 10, blank);

  for(unsigned int i = 0; i < map.getSizeInCellsX(); ++i){
    for(unsigned int j = 0; j < map.getSizeInCellsY(); ++j){
      ASSERT_EQ(map.getCost(i, j), 0);
    }
  }

  std::vector<unsigned char> fully_contained(25);
  fully_contained[0] = 254;
  fully_contained[4] = 254;
  fully_contained[5] = 254;
  fully_contained[9] = 254;

  Costmap2D small_static_map(5, 5, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
      10.0, MAX_Z, 10.0, 25, fully_contained, THRESHOLD);

  map.updateStaticMapWindow(0, 0, 5, 5, fully_contained);

  ASSERT_FLOAT_EQ(map.getOriginX(), -10);
  ASSERT_FLOAT_EQ(map.getOriginX(), -10);
  ASSERT_EQ(map.getSizeInCellsX(), (unsigned int)15);
  ASSERT_EQ(map.getSizeInCellsY(), (unsigned int)15);
  for(unsigned int j = 0; j < 5; ++j){
    for(unsigned int i = 0; i < 5; ++i){
      ASSERT_EQ(map.getCost(i + 10, j + 10), small_static_map.getCost(i, j));
    }
  }

}


TEST(costmap, testStaticMap){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS, 
      10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  ASSERT_EQ(map.getSizeInCellsX(), (unsigned int)10);
  ASSERT_EQ(map.getSizeInCellsY(), (unsigned int)10);

  // Verify that obstacles correctly identified from the static map.
  std::vector<unsigned int> occupiedCells;

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE){
        occupiedCells.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(occupiedCells.size(), (unsigned int)20);

  // Iterate over all id's and verify that they are present according to their
  for(std::vector<unsigned int>::const_iterator it = occupiedCells.begin(); it != occupiedCells.end(); ++it){
    unsigned int ind = *it;
    unsigned int x, y;
    map.indexToCells(ind, x, y);
    ASSERT_EQ(find(occupiedCells, map.getIndex(x, y)), true);
    ASSERT_EQ(MAP_10_BY_10[ind] >= 100, true);
    ASSERT_EQ(map.getCost(x, y) >= 100, true);
  }

  // Block of 200
  ASSERT_EQ(find(occupiedCells, map.getIndex(7, 2)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(8, 2)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(9, 2)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(7, 3)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(8, 3)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(9, 3)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(7, 4)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(8, 4)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(9, 4)), true);

  // Block of 100
  ASSERT_EQ(find(occupiedCells, map.getIndex(4, 3)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(4, 4)), true);

  // Block of 200
  ASSERT_EQ(find(occupiedCells, map.getIndex(3, 7)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(4, 7)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(5, 7)), true);


  // Verify Coordinate Transformations, ROW MAJOR ORDER
  ASSERT_EQ(worldToIndex(map, 0.0, 0.0), (unsigned int)0);
  ASSERT_EQ(worldToIndex(map, 0.0, 0.99), (unsigned int)0);
  ASSERT_EQ(worldToIndex(map, 0.0, 1.0), (unsigned int)10);
  ASSERT_EQ(worldToIndex(map, 1.0, 0.99), (unsigned int)1);
  ASSERT_EQ(worldToIndex(map, 9.99, 9.99), (unsigned int)99);
  ASSERT_EQ(worldToIndex(map, 8.2, 3.4), (unsigned int)38);

  // Ensure we hit the middle of the cell for world co-ordinates
  double wx, wy;
  indexToWorld(map, 99, wx, wy);
  ASSERT_EQ(wx, 9.5);
  ASSERT_EQ(wy, 9.5);
}

//*/


int main(int argc, char** argv){
  ros::init(argc, argv, "obstacle_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
