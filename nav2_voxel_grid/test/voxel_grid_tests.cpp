/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <nav2_voxel_grid/voxel_grid.hpp>
#include <gtest/gtest.h>

TEST(voxel_grid, basicMarkingAndClearing) {
  int size_x = 50, size_y = 10, size_z = 16;
  nav2_voxel_grid::VoxelGrid vg(size_x, size_y, size_z);

  // Put a "tabletop" into the scene.  A flat rectangle of set voxels at z = 12.
  int table_z = 12;
  int table_x_min = 5, table_x_max = 15;
  int table_y_min = 0, table_y_max = 3;
  for (int x = table_x_min; x <= table_x_max; x++) {
    vg.markVoxelLine(x, table_y_min, table_z, x, table_y_max, table_z);
  }

  for (int i = table_x_min; i <= table_x_max; ++i) {
    for (int j = table_y_min; j <= table_y_max; ++j) {
      // check that each cell of the table is marked
      ASSERT_EQ(nav2_voxel_grid::MARKED, vg.getVoxel(i, j, table_z));
    }
  }

  int mark_count = 0;
  unsigned int unknown_count = 0;
  // go through each cell in the voxel grid and make sure that only 44 are filled in
  for (unsigned int i = 0; i < vg.sizeX(); ++i) {
    for (unsigned int j = 0; j < vg.sizeY(); ++j) {
      for (unsigned int k = 0; k < vg.sizeZ(); ++k) {
        if (vg.getVoxel(i, j, k) == nav2_voxel_grid::MARKED) {
          mark_count++;
        } else if (vg.getVoxel(i, j, k) == nav2_voxel_grid::UNKNOWN) {
          unknown_count++;
        }
      }
    }
  }
  ASSERT_EQ(mark_count, 44);

  // the rest of the cells should be unknown
  ASSERT_EQ(unknown_count, vg.sizeX() * vg.sizeY() * vg.sizeZ() - 44);

  // now, let's clear one of the rows of the table
  vg.clearVoxelLine(table_x_min, table_y_min, table_z, table_x_max, table_y_min, table_z);

  mark_count = 0;
  unknown_count = 0;
  int free_count = 0;
  // go through each cell in the voxel grid and make sure that only 33 are now filled in
  for (unsigned int i = 0; i < vg.sizeX(); ++i) {
    for (unsigned int j = 0; j < vg.sizeY(); ++j) {
      for (unsigned int k = 0; k < vg.sizeZ(); ++k) {
        if (vg.getVoxel(i, j, k) == nav2_voxel_grid::MARKED) {
          mark_count++;
        } else if (vg.getVoxel(i, j, k) == nav2_voxel_grid::FREE) {
          free_count++;
        } else if (vg.getVoxel(i, j, k) == nav2_voxel_grid::UNKNOWN) {
          unknown_count++;
        }
      }
    }
  }

  // we've switched 11 cells from marked to free
  ASSERT_EQ(mark_count, 33);

  // we've just explicitly seen through 11 cells
  ASSERT_EQ(free_count, 11);

  // the rest of the cells should still be unknown
  ASSERT_EQ(unknown_count, vg.sizeX() * vg.sizeY() * vg.sizeZ() - 44);

  // now let's put in a vertical column manually to test markVoxel
  for (unsigned int i = 0; i < vg.sizeZ(); ++i) {
    vg.markVoxel(0, 0, i);
    ASSERT_EQ(vg.getVoxel(0, 0, i), nav2_voxel_grid::MARKED);
  }

  vg.printColumnGrid();
  vg.printVoxelGrid();

  // now, let's clear that line of voxels and make sure that they clear out OK
  vg.clearVoxelLine(0, 0, 0, 0, 0, vg.sizeZ() - 1);

  for (unsigned int i = 0; i < vg.sizeZ(); ++i) {
    ASSERT_EQ(vg.getVoxel(0, 0, i), nav2_voxel_grid::FREE);
  }

  mark_count = 0;

  // Visualize the output
  /*
     v->printVoxelGrid();
     v->printColumnGrid();

     printf("CostMap:\n===========\n");
     for(int y = 0; y < size_y; y++){
     for(int x = 0; x < size_x; x++){
     printf((costMap[y * size_x + x] > 0 ? "#" : " "));
     }printf("|\n");
     }
     */
}

TEST(voxel_grid, InvalidSize) {
  int size_x = 50, size_y = 10, size_z = 17;
  int test_z = 16;
  nav2_voxel_grid::VoxelGrid vg(size_x, size_y, size_z);
  vg.resize(size_x, size_y, test_z);
  vg.resize(size_x, size_y, size_z);
  EXPECT_TRUE(vg.getVoxelColumn(51, 10, 0, 0) == nav2_voxel_grid::VoxelStatus::UNKNOWN);
  EXPECT_TRUE(vg.getVoxelColumn(50, 11, 0, 0) == nav2_voxel_grid::VoxelStatus::UNKNOWN);
}

TEST(voxel_grid, MarkAndClear) {
  int size_x = 10, size_y = 10, size_z = 10;
  nav2_voxel_grid::VoxelGrid vg(size_x, size_y, size_z);
  vg.markVoxelInMap(5, 5, 5, 0);
  EXPECT_EQ(vg.getVoxel(5, 5, 5), nav2_voxel_grid::MARKED);
  vg.clearVoxelColumn(55);
  EXPECT_EQ(vg.getVoxel(5, 5, 5), nav2_voxel_grid::FREE);
}

TEST(voxel_grid, clearVoxelLineInMap) {
  int size_x = 10, size_y = 10, size_z = 10;
  nav2_voxel_grid::VoxelGrid vg(size_x, size_y, size_z);
  vg.markVoxelInMap(0, 0, 5, 0);
  EXPECT_EQ(vg.getVoxel(0, 0, 5), nav2_voxel_grid::MARKED);

  unsigned char * map_2d = new unsigned char[100];
  map_2d[0] = 254;

  vg.clearVoxelLineInMap(0, 0, 0, 0, 0, 9, map_2d, 16, 0);

  EXPECT_EQ(map_2d[0], 0);

  vg.markVoxelInMap(0, 0, 5, 0);
  vg.clearVoxelLineInMap(0, 0, 0, 0, 0, 9, nullptr, 16, 0);
  EXPECT_EQ(vg.getVoxel(0, 0, 5), nav2_voxel_grid::FREE);
  delete[] map_2d;
}

TEST(voxel_grid, GetVoxelData) {
  uint32_t * data = new uint32_t[9];
  data[4] = 255;
  data[0] = 0;
  EXPECT_EQ(
    nav2_voxel_grid::VoxelGrid::getVoxel(1, 1, 1, 3, 3, 3, data), nav2_voxel_grid::UNKNOWN);

  EXPECT_EQ(
    nav2_voxel_grid::VoxelGrid::getVoxel(0, 0, 0, 3, 3, 3, data), nav2_voxel_grid::FREE);
  delete[] data;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
