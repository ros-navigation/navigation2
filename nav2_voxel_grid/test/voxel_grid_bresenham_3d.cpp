/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021 Samsung Research Russia
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
* Author: Alexey Merzlyakov
*********************************************************************/
#include <nav2_voxel_grid/voxel_grid.hpp>
#include <gtest/gtest.h>

class TestVoxel
{
public:
  explicit TestVoxel(uint32_t * data, int sz_x, int sz_y)
  : data_(data)
  {
    size_ = sz_x * sz_y;
  }
  inline void operator()(unsigned int off, unsigned int val)
  {
    ASSERT_TRUE(off < size_);
    data_[off] = val;
  }
  inline unsigned int operator()(unsigned int off)
  {
    return data_[off];
  }

private:
  uint32_t * data_;
  unsigned int size_;
};

TEST(voxel_grid, bresenham3DBoundariesCheck)
{
  const int sz_x = 60;
  const int sz_y = 60;
  const int sz_z = 2;
  const unsigned int max_length = 60;
  const unsigned int min_length = 6;
  nav2_voxel_grid::VoxelGrid vg(sz_x, sz_y, sz_z);
  TestVoxel tv(vg.getData(), sz_x, sz_y);

  // Initial point - some assymetrically standing point in order to cover most corner cases
  const double x0 = 2.2;
  const double y0 = 3.8;
  const double z0 = 0.4;
  // z-axis won't be domimant
  const double z1 = 0.5;
  // (x1, y1) point will move
  double x1, y1;

  // Epsilon for outer boundaries of voxel grid array
  const double epsilon = 0.02;

  // Running on (x, 0) edge
  y1 = 0.0;
  for (int i = 0; i <= sz_x; i++) {
    if (i != sz_x) {
      x1 = i;
    } else {
      x1 = i - epsilon;
    }
    vg.raytraceLine(tv, x0, y0, z0, x1, y1, z1, max_length, min_length);
  }

  // Running on (x, sz_y) edge
  y1 = sz_y - epsilon;
  for (int i = 0; i <= sz_x; i++) {
    if (i != sz_x) {
      x1 = i;
    } else {
      x1 = i - epsilon;
    }
    vg.raytraceLine(tv, x0, y0, z0, x1, y1, z1, max_length, min_length);
  }

  // Running on (0, y) edge
  x1 = 0.0;
  for (int j = 0; j <= sz_y; j++) {
    if (j != sz_y) {
      y1 = j;
    } else {
      y1 = j - epsilon;
    }
    vg.raytraceLine(tv, x0, y0, z0, x1, y1, z1, max_length, min_length);
  }

  // Running on (sz_x, y) edge
  x1 = sz_x - epsilon;
  for (int j = 0; j <= sz_y; j++) {
    if (j != sz_y) {
      y1 = j;
    } else {
      y1 = j - epsilon;
    }
    vg.raytraceLine(tv, x0, y0, z0, x1, y1, z1, max_length, min_length);
  }
}

TEST(voxel_grid, bresenham3DSamePoint)
{
  const int sz_x = 60;
  const int sz_y = 60;
  const int sz_z = 2;
  const unsigned int max_length = 60;
  const unsigned int min_length = 0;
  nav2_voxel_grid::VoxelGrid vg(sz_x, sz_y, sz_z);
  TestVoxel tv(vg.getData(), sz_x, sz_y);

  // Initial point
  const double x0 = 2.2;
  const double y0 = 3.8;
  const double z0 = 0.4;

  unsigned int offset = int(y0) * sz_x + int(x0);
  unsigned int val_before = tv(offset);
  // Same point to check
  vg.raytraceLine(tv, x0, y0, z0, x0, y0, z0, max_length, min_length);
  unsigned int val_after = tv(offset);
  ASSERT_FALSE(val_before == val_after);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
