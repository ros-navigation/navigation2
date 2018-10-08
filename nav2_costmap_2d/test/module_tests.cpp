/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 * @author Conor McGann
 * Test harness for Costmap2D
 */

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/observation_buffer.h>
#include <set>
#include <gtest/gtest.h>

using namespace costmap_2d;

const unsigned char MAP_10_BY_10_CHAR[] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 200, 200, 200,
  0, 0, 0, 0, 100, 0, 0, 200, 200, 200,
  0, 0, 0, 0, 100, 0, 0, 200, 200, 200,
  70, 70, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 200, 200, 200, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 255, 255, 255,
  0, 0, 0, 0, 0, 0, 0, 255, 255, 255
};

const unsigned char MAP_5_BY_5_CHAR[] = {
  0, 0, 0, 0, 0,
  0, 0, 0, 0, 0,
  0, 0, 0, 0, 0,
  0, 0, 0, 0, 0,
  0, 0, 0, 0, 0,
};

std::vector<unsigned char> MAP_5_BY_5;
std::vector<unsigned char> MAP_10_BY_10;
std::vector<unsigned char> EMPTY_10_BY_10;
std::vector<unsigned char> EMPTY_100_BY_100;

const unsigned int GRID_WIDTH(10);
const unsigned int GRID_HEIGHT(10);
const double RESOLUTION(1);
const double WINDOW_LENGTH(10);
const unsigned char THRESHOLD(100);
const double MAX_Z(1.0);
const double RAYTRACE_RANGE(20.0);
const double OBSTACLE_RANGE(20.0);
const double ROBOT_RADIUS(1.0);

bool find(const std::vector<unsigned int>& l, unsigned int n){
  for(std::vector<unsigned int>::const_iterator it = l.begin(); it != l.end(); ++it){
    if(*it == n)
      return true;
  }

  return false;
}

/**
 * Tests the reset method
 */
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

/**
 * Test for the cost function correctness with a larger range and different values
 */
TEST(costmap, testCostFunctionCorrectness){
  Costmap2D map(100, 100, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS * 5.0, ROBOT_RADIUS * 8.0, ROBOT_RADIUS * 10.5, 
      100.0, MAX_Z, 100.0, 25, EMPTY_100_BY_100, THRESHOLD);

  // Verify that the circumscribed cost lower bound is as expected: based on the cost function.
  unsigned char c = map.computeCost((ROBOT_RADIUS * 8.0 / RESOLUTION));
  ASSERT_EQ(map.getCircumscribedCost(), c);

  // Add a point in the center
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(1);
  cloud.points[0].x = 50;
  cloud.points[0].y = 50;
  cloud.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, cloud, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  for(unsigned int i = 0; i <= (unsigned int)ceil(ROBOT_RADIUS * 5.0); i++){
    // To the right
    ASSERT_EQ(map.getCost(50 + i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map.getCost(50 + i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // To the left
    ASSERT_EQ(map.getCost(50 - i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map.getCost(50 - i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // Down
    ASSERT_EQ(map.getCost(50, 50 + i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map.getCost(50, 50 + i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // Up
    ASSERT_EQ(map.getCost(50, 50 - i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map.getCost(50, 50 - i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
  }

  // Verify the normalized cost attenuates as expected
  for(unsigned int i = (unsigned int)(ceil(ROBOT_RADIUS * 5.0) + 1); i <= (unsigned int)ceil(ROBOT_RADIUS * 10.5); i++){
    unsigned char expectedValue = map.computeCost(i / RESOLUTION);
    ASSERT_EQ(map.getCost(50 + i, 50), expectedValue);
  }

  // Update with no hits. Should clear (revert to the static map
  map.resetMapOutsideWindow(0, 0, 0.0, 0.0);
  cloud.points.resize(0);

  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs2(p, cloud, 100.0, 100.0);
  std::vector<Observation> obsBuf2;
  obsBuf2.push_back(obs2);

  map.updateWorld(0, 0, obsBuf2, obsBuf2);

  for(unsigned int i = 0; i < 100; i++)
    for(unsigned int j = 0; j < 100; j++)
      ASSERT_EQ(map.getCost(i, j), costmap_2d::FREE_SPACE);
}

char printableCost( unsigned char cost )
{
  switch( cost )
  {
  case NO_INFORMATION: return '?';
  case LETHAL_OBSTACLE: return 'L';
  case INSCRIBED_INFLATED_OBSTACLE: return 'I';
  case FREE_SPACE: return '.';
  default: return '0' + (unsigned char) (10 * cost / 255);
  }
}

/**
 * Test for wave interference
 */
TEST(costmap, testWaveInterference){
  // Start with an empty map
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS * 2, ROBOT_RADIUS * 3.01,
      10.0, MAX_Z * 2, 10.0, 1, EMPTY_10_BY_10, THRESHOLD);

  // Lay out 3 obstacles in a line - along the diagonal, separated by a cell.
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(3);
  cloud.points[0].x = 3;
  cloud.points[0].y = 3;
  cloud.points[0].z = MAX_Z;
  cloud.points[1].x = 5;
  cloud.points[1].y = 5;
  cloud.points[1].z = MAX_Z;
  cloud.points[2].x = 7;
  cloud.points[2].y = 7;
  cloud.points[2].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, cloud, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  int update_count = 0;

  // Expect to see a union of obstacles
  printf("map:\n");
  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) != costmap_2d::FREE_SPACE){
        update_count++;
      }
      printf("%c", printableCost( map.getCost( i, j )));
    }
    printf("\n");
  }

  ASSERT_EQ(update_count, 79);
}

/** Test for copying a window of a costmap */
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
  */

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

/**
 * Test for ray tracing free space
 */
TEST(costmap, testRaytracing){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS, 
      10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  // Add a point cloud, should not affect the map
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(1);
  cloud.points[0].x = 0;
  cloud.points[0].y = 0;
  cloud.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, cloud, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  int lethal_count = 0;

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE){
        lethal_count++;
      }
    }
  }

  //we expect just one obstacle to be added
  ASSERT_EQ(lethal_count, 21);
}

TEST(costmap, testAdjacentToObstacleCanStillMove){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, 2.1, 3.1, 4.1, 
                10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(1);
  cloud.points[0].x = 0;
  cloud.points[0].y = 0;
  cloud.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, cloud, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(9, 9, obsBuf, obsBuf);

  EXPECT_EQ( LETHAL_OBSTACLE, map.getCost( 0, 0 ));
  EXPECT_EQ( INSCRIBED_INFLATED_OBSTACLE, map.getCost( 1, 0 ));
  EXPECT_EQ( INSCRIBED_INFLATED_OBSTACLE, map.getCost( 2, 0 ));
  EXPECT_TRUE( INSCRIBED_INFLATED_OBSTACLE > map.getCost( 3, 0 ));
  EXPECT_TRUE( INSCRIBED_INFLATED_OBSTACLE > map.getCost( 2, 1 ));
  EXPECT_EQ( INSCRIBED_INFLATED_OBSTACLE, map.getCost( 1, 1 ));
}

TEST(costmap, testInflationShouldNotCreateUnknowns){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, 2.1, 3.1, 4.1, 
                10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(1);
  cloud.points[0].x = 0;
  cloud.points[0].y = 0;
  cloud.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, cloud, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(9, 9, obsBuf, obsBuf);

  int unknown_count = 0;

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::NO_INFORMATION){
        unknown_count++;
      }
    }
  }
  EXPECT_EQ( 0, unknown_count );
}

unsigned int worldToIndex(Costmap2D& map, double wx, double wy){
  unsigned int mx, my;
  map.worldToMap(wx, wy, mx, my);
  return map.getIndex(mx, my);
}

void indexToWorld(Costmap2D& map, unsigned int index, double& wx, double& wy){
  unsigned int mx, my;
  map.indexToCells(index, mx, my);
  map.mapToWorld(mx, my, wx, wy);
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


/**
 * Verify that dynamic obstacles are added
 */

TEST(costmap, testDynamicObstacles){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS, 
      10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  // Add a point cloud and verify its insertion. There should be only one new one
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(3);
  cloud.points[0].x = 0;
  cloud.points[0].y = 0;
  cloud.points[1].x = 0;
  cloud.points[1].y = 0;
  cloud.points[2].x = 0;
  cloud.points[2].y = 0;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, cloud, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  std::vector<unsigned int> ids;

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE){
        ids.push_back(map.getIndex(i, j));
      }
    }
  }

  // Should now have 1 insertion and no deletions
  ASSERT_EQ(ids.size(), (unsigned int)21);

  // Repeating the call - we should see no insertions or deletions
  map.updateWorld(0, 0, obsBuf, obsBuf);
  ASSERT_EQ(ids.size(), (unsigned int)21);
}

/**
 * Verify that if we add a point that is already a static obstacle we do not end up with a new ostacle
 */
TEST(costmap, testMultipleAdditions){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS, 
      10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  // A point cloud with one point that falls within an existing obstacle
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(1);
  cloud.points[0].x = 7;
  cloud.points[0].y = 2;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, cloud, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  std::vector<unsigned int> ids;

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE){
        ids.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(ids.size(), (unsigned int)20);
}

/**
 * Make sure we ignore points outside of our z threshold
 */
TEST(costmap, testZThreshold){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS, 
      10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  // A point cloud with 2 points falling in a cell with a non-lethal cost
  pcl::PointCloud<pcl::PointXYZ> c0;
  c0.points.resize(2);
  c0.points[0].x = 0;
  c0.points[0].y = 5;
  c0.points[0].z = 0.4;
  c0.points[1].x = 1;
  c0.points[1].y = 5;
  c0.points[1].z = 1.2;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, c0, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  std::vector<unsigned int> ids;

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE){
        ids.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(ids.size(), (unsigned int)21);
}

/**
 * Test inflation for both static and dynamic obstacles
 */

TEST(costmap, testInflation){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS, 
      10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  // Verify that obstacles correctly identified
  std::vector<unsigned int> occupiedCells;

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE || map.getCost(i, j) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        occupiedCells.push_back(map.getIndex(i, j));
      }
    }
  }

  // There should be no duplicates
  std::set<unsigned int> setOfCells;
  for(unsigned int i=0;i<occupiedCells.size(); i++)
    setOfCells.insert(i);

  ASSERT_EQ(setOfCells.size(), occupiedCells.size());
  ASSERT_EQ(setOfCells.size(), (unsigned int)48);

  // Iterate over all id's and verify they are obstacles
  for(std::vector<unsigned int>::const_iterator it = occupiedCells.begin(); it != occupiedCells.end(); ++it){
    unsigned int ind = *it;
    unsigned int x, y;
    map.indexToCells(ind, x, y);
    ASSERT_EQ(find(occupiedCells, map.getIndex(x, y)), true);
    ASSERT_EQ(map.getCost(x, y) == costmap_2d::LETHAL_OBSTACLE || map.getCost(x, y) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
  }

  // Set an obstacle at the origin and observe insertions for it and its neighbors
  pcl::PointCloud<pcl::PointXYZ> c0;
  c0.points.resize(1);
  c0.points[0].x = 0;
  c0.points[0].y = 0;
  c0.points[0].z = 0.4;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, c0, 100.0, 100.0);
  std::vector<Observation> obsBuf, empty;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, empty);

  occupiedCells.clear();
  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE || map.getCost(i, j) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        occupiedCells.push_back(map.getIndex(i, j));
      }
    }
  }

  // It and its 2 neighbors makes 3 obstacles
  ASSERT_EQ(occupiedCells.size(), (unsigned int)51);

  // @todo Rewrite 
  // Add an obstacle at <2,0> which will inflate and refresh to of the other inflated cells
  pcl::PointCloud<pcl::PointXYZ> c1;
  c1.points.resize(1);
  c1.points[0].x = 2;
  c1.points[0].y = 0;
  c1.points[0].z = 0.0;

  geometry_msgs::Point p1;
  p1.x = 0.0;
  p1.y = 0.0;
  p1.z = MAX_Z;

  Observation obs1(p1, c1, 100.0, 100.0);
  std::vector<Observation> obsBuf1;
  obsBuf1.push_back(obs1);

  map.updateWorld(0, 0, obsBuf1, empty);

  occupiedCells.clear();
  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE || map.getCost(i, j) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        occupiedCells.push_back(map.getIndex(i, j));
      }
    }
  }

  // Now we expect insertions for it, and 2 more neighbors, but not all 5. Free space will propagate from
  // the origin to the target, clearing the point at <0, 0>, but not over-writing the inflation of the obstacle
  // at <0, 1>
  ASSERT_EQ(occupiedCells.size(), (unsigned int)54);


  // Add an obstacle at <1, 9>. This will inflate obstacles around it
  pcl::PointCloud<pcl::PointXYZ> c2;
  c2.points.resize(1);
  c2.points[0].x = 1;
  c2.points[0].y = 9;
  c2.points[0].z = 0.0;

  geometry_msgs::Point p2;
  p2.x = 0.0;
  p2.y = 0.0;
  p2.z = MAX_Z;

  Observation obs2(p2, c2, 100.0, 100.0);
  std::vector<Observation> obsBuf2;
  obsBuf2.push_back(obs2);

  map.updateWorld(0, 0, obsBuf2, empty);

  ASSERT_EQ(map.getCost(1, 9), costmap_2d::LETHAL_OBSTACLE);
  ASSERT_EQ(map.getCost(0, 9), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  ASSERT_EQ(map.getCost(2, 9), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  // Add an obstacle and verify that it over-writes its inflated status
  pcl::PointCloud<pcl::PointXYZ> c3;
  c3.points.resize(1);
  c3.points[0].x = 0;
  c3.points[0].y = 9;
  c3.points[0].z = 0.0;

  geometry_msgs::Point p3;
  p3.x = 0.0;
  p3.y = 0.0;
  p3.z = MAX_Z;

  Observation obs3(p3, c3, 100.0, 100.0);
  std::vector<Observation> obsBuf3;
  obsBuf3.push_back(obs3);

  map.updateWorld(0, 0, obsBuf3, empty);

  ASSERT_EQ(map.getCost(0, 9), costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test specific inflation scenario to ensure we do not set inflated obstacles to be raw obstacles.
 */
TEST(costmap, testInflation2){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS, 
      10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  // Creat a small L-Shape all at once
  pcl::PointCloud<pcl::PointXYZ> c0;
  c0.points.resize(3);
  c0.points[0].x = 1;
  c0.points[0].y = 1;
  c0.points[0].z = MAX_Z;
  c0.points[1].x = 1;
  c0.points[1].y = 2;
  c0.points[1].z = MAX_Z;
  c0.points[2].x = 2;
  c0.points[2].y = 2;
  c0.points[2].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, c0, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  ASSERT_EQ(map.getCost(3, 2), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);  
  ASSERT_EQ(map.getCost(3, 3), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

/**
 * Test inflation behavior, starting with an empty map
 */
TEST(costmap, testInflation3){
  std::vector<unsigned char> mapData;
  for(unsigned int i=0; i< GRID_WIDTH; i++){
    for(unsigned int j = 0; j < GRID_HEIGHT; j++){
      mapData.push_back(0);
    }
  }

  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS * 2, ROBOT_RADIUS * 3, 
      10.0, MAX_Z, 10.0, 1, mapData, THRESHOLD);

  // There should be no occupied cells
  std::vector<unsigned int> ids;

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE || map.getCost(i, j) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        ids.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(ids.size(), (unsigned int)0);

  // Add an obstacle at 5,5
  pcl::PointCloud<pcl::PointXYZ> c0;
  c0.points.resize(1);
  c0.points[0].x = 5;
  c0.points[0].y = 5;
  c0.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, c0, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) != costmap_2d::FREE_SPACE){
        ids.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(ids.size(), (unsigned int)29);

  ids.clear();
  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE || map.getCost(i, j) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        ids.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(ids.size(), (unsigned int)5);

  // Update again - should see no change
  map.updateWorld(0, 0, obsBuf, obsBuf);

  ids.clear();
  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) != costmap_2d::FREE_SPACE){
        ids.push_back(map.getIndex(i, j));
      }
    }
  }
  
  ASSERT_EQ(ids.size(), (unsigned int)29);
}

/**
 * Test for ray tracing free space
 */

TEST(costmap, testRaytracing2){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS, 
      100.0, MAX_Z, 100.0, 1, MAP_10_BY_10, THRESHOLD);

  // The sensor origin will be <0,0>. So if we add an obstacle at 9,9, we would expect cells
  // <0, 0> thru <8, 8> to be traced through
  pcl::PointCloud<pcl::PointXYZ> c0;
  c0.points.resize(1);
  c0.points[0].x = 9.5;
  c0.points[0].y = 9.5;
  c0.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.5;
  p.y = 0.5;
  p.z = MAX_Z;

  Observation obs(p, c0, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  std::vector<unsigned int> obstacles;

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE){
        obstacles.push_back(map.getIndex(i, j));
      }
    }
  }

  unsigned int obs_before = obstacles.size();

  map.updateWorld(0, 0, obsBuf, obsBuf);

  obstacles.clear();
  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE){
        obstacles.push_back(map.getIndex(i, j));
      }
    }
  }

  //Two obstacles shoulb be removed from the map by raytracing
  ASSERT_EQ(obstacles.size(), obs_before - 2);


  // many cells will have been switched to free space along the diagonal except
  // for those inflated in the update.. tests that inflation happens properly
  // after raytracing
  unsigned char test[10]= {0, 0, 0, 253, 253, 0, 0, 253, 253, 254};
  for(unsigned int i=0; i < 10; i++)
    ASSERT_EQ(map.getCost(i, i), test[i]);
}

/**
 * Within a certian radius of the robot, the cost map most propagate obstacles. This
 * is to avoid a case where a hit on a far obstacle clears inscribed radius around a
 * near one.
 */

TEST(costmap, testTrickyPropagation){
  const unsigned char MAP_HALL_CHAR[10 * 10] = {
    0,   0, 0,   0,   0, 0,   0, 0, 0, 0,
    254, 0, 0,   0,   0, 0,   0, 0, 0, 0,
    0,   0, 0,   0,   0, 0,   0, 0, 0, 0,
    0,   0, 0,   254, 0, 0,   0, 0, 0, 0,
    0,   0, 0,   0,   0, 0,   0, 0, 0, 0,
    0,   0, 0,   0,   0, 0,   0, 0, 0, 0,
    0,   0, 0,   0,   0, 254, 0, 0, 0, 0,
    0,   0, 0,   0,   0, 254, 0, 0, 0, 0,
    0,   0, 0,   0,   0, 0,   0, 0, 0, 0,
    0,   0, 0,   0,   0, 0,   0, 0, 0, 0,
  };
  std::vector<unsigned char> MAP_HALL;
  for (int i = 0; i < 10 * 10; i++) {
    MAP_HALL.push_back(MAP_HALL_CHAR[i]);
  }

  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS, 
      100.0, MAX_Z, 100.0, 1, MAP_HALL, THRESHOLD);


  //Add a dynamic obstacle
  pcl::PointCloud<pcl::PointXYZ> c2;
  c2.points.resize(3);
  //Dynamic obstacle that raytaces.
  c2.points[0].x = 7.0;
  c2.points[0].y = 8.0;
  c2.points[0].z = 1.0;
  //Dynamic obstacle that should not be raytraced the
  //first update, but should on the second.
  c2.points[1].x = 3.0;
  c2.points[1].y = 4.0;
  c2.points[1].z = 1.0;
  //Dynamic obstacle that should not be erased.
  c2.points[2].x = 6.0;
  c2.points[2].y = 3.0;
  c2.points[2].z = 1.0;

  geometry_msgs::Point p2;
  p2.x = 0.5;
  p2.y = 0.5;
  p2.z = MAX_Z;

  Observation obs2(p2, c2, 100.0, 100.0);
  std::vector<Observation> obsBuf2;
  obsBuf2.push_back(obs2);

  map.updateWorld(0, 0, obsBuf2, obsBuf2);

  const unsigned char MAP_HALL_CHAR_TEST[10 * 10] = { 
    253, 254, 253,   0,   0,   0,   0,   0,   0,   0,
      0, 253,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0, 253,   0,   0,   0,   0,   0,
      0,   0,   0, 253, 254, 253,   0,   0,   0,   0,
      0,   0,   0,   0, 253,   0,   0, 253,   0,   0,
      0,   0,   0, 253,   0,   0, 253, 254, 253,   0,
      0,   0, 253, 254, 253,   0,   0, 253, 253,   0,
      0,   0,   0, 253,   0,   0,   0, 253, 254, 253,
      0,   0,   0,   0,   0,   0,   0,   0, 253,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  };

  
  for (int i = 0; i < 10 * 10; i++) {
    ASSERT_EQ(map.getCost(i / 10, i % 10), MAP_HALL_CHAR_TEST[i]);
  }

  pcl::PointCloud<pcl::PointXYZ> c;
  c.points.resize(1);
  //Dynamic obstacle that raytaces the one at (3.0, 4.0).
  c.points[0].x = 4.0;
  c.points[0].y = 5.0;
  c.points[0].z = 1.0;

  geometry_msgs::Point p3;
  p3.x = 0.5;
  p3.y = 0.5;
  p3.z = MAX_Z;

  Observation obs3(p3, c, 100.0, 100.0);
  std::vector<Observation> obsBuf3;
  obsBuf3.push_back(obs3);

  map.updateWorld(0, 0, obsBuf3, obsBuf3);

  const unsigned char MAP_HALL_CHAR_TEST2[10 * 10] = { 
    253, 254, 253,   0,   0,   0,   0,   0,   0,   0,
      0, 253,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0, 253,   0,   0,   0,   0,
      0,   0,   0,   0, 253, 254, 253, 253,   0,   0,
      0,   0,   0, 253,   0, 253, 253, 254, 253,   0,
      0,   0, 253, 254, 253,   0,   0, 253, 253,   0,
      0,   0,   0, 253,   0,   0,   0, 253, 254, 253,
      0,   0,   0,   0,   0,   0,   0,   0, 253,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  };

  
  for (int i = 0; i < 10 * 10; i++) {
    ASSERT_EQ(map.getCost(i / 10, i % 10), MAP_HALL_CHAR_TEST2[i]);
  }
}



int main(int argc, char** argv){
  for(unsigned int i = 0; i< GRID_WIDTH * GRID_HEIGHT; i++){
    EMPTY_10_BY_10.push_back(0);
    MAP_10_BY_10.push_back(MAP_10_BY_10_CHAR[i]);
  }

  for(unsigned int i = 0; i< 5 * 5; i++){
    MAP_5_BY_5.push_back(MAP_10_BY_10_CHAR[i]);
  }

  for(unsigned int i = 0; i< 100 * 100; i++)
    EMPTY_100_BY_100.push_back(0);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
