/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2022 Samsung Research Russia
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
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <gtest/gtest.h>

class CostmapAction
{
public:
  explicit CostmapAction(
    unsigned char * costmap, unsigned int size, unsigned char mark_val = 128)
  : costmap_(costmap), size_(size), mark_val_(mark_val)
  {
  }

  inline void operator()(unsigned int off)
  {
    ASSERT_TRUE(off < size_);
    costmap_[off] = mark_val_;
  }

  inline unsigned int get(unsigned int off)
  {
    return costmap_[off];
  }

private:
  unsigned char * costmap_;
  unsigned int size_;
  unsigned char mark_val_;
};

class CostmapTest : public nav2_costmap_2d::Costmap2D
{
public:
  CostmapTest(
    unsigned int size_x, unsigned int size_y, double resolution,
    double origin_x, double origin_y, unsigned char default_val = 0)
  : nav2_costmap_2d::Costmap2D(size_x, size_y, resolution, origin_x, origin_y, default_val)
  {
  }

  unsigned char * getCostmap()
  {
    return costmap_;
  }

  unsigned int getSize()
  {
    return size_x_ * size_y_;
  }

  void raytraceLine(
    CostmapAction ca, unsigned int x0, unsigned int y0, unsigned int x1,
    unsigned int y1,
    unsigned int max_length = UINT_MAX, unsigned int min_length = 0)
  {
    nav2_costmap_2d::Costmap2D::raytraceLine(ca, x0, y0, x1, y1, max_length, min_length);
  }
};

TEST(costmap_2d, bresenham2DBoundariesCheck)
{
  const unsigned int sz_x = 60;
  const unsigned int sz_y = 60;
  const unsigned int max_length = 60;
  const unsigned int min_length = 6;
  CostmapTest ct(sz_x, sz_y, 0.1, 0.0, 0.0);
  CostmapAction ca(ct.getCostmap(), ct.getSize());

  // Initial point - some assymetrically standing point in order to cover most corner cases
  const unsigned int x0 = 2;
  const unsigned int y0 = 4;
  // (x1, y1) point will move
  unsigned int x1, y1;

  // Running on (x, 0) edge
  y1 = 0;
  for (x1 = 0; x1 < sz_x; x1++) {
    ct.raytraceLine(ca, x0, y0, x1, y1, max_length, min_length);
  }

  // Running on (x, sz_y) edge
  y1 = sz_y - 1;
  for (x1 = 0; x1 < sz_x; x1++) {
    ct.raytraceLine(ca, x0, y0, x1, y1, max_length, min_length);
  }

  // Running on (0, y) edge
  x1 = 0;
  for (y1 = 0; y1 < sz_y; y1++) {
    ct.raytraceLine(ca, x0, y0, x1, y1, max_length, min_length);
  }

  // Running on (sz_x, y) edge
  x1 = sz_x - 1;
  for (y1 = 0; y1 < sz_y; y1++) {
    ct.raytraceLine(ca, x0, y0, x1, y1, max_length, min_length);
  }
}

TEST(costmap_2d, bresenham2DSamePoint)
{
  const unsigned int sz_x = 60;
  const unsigned int sz_y = 60;
  const unsigned int max_length = 60;
  const unsigned int min_length = 0;
  CostmapTest ct(sz_x, sz_y, 0.1, 0.0, 0.0);
  CostmapAction ca(ct.getCostmap(), ct.getSize());

  // Initial point
  const double x0 = 2;
  const double y0 = 4;

  unsigned int offset = y0 * sz_x + x0;
  unsigned char val_before = ca.get(offset);
  // Same point to check
  ct.raytraceLine(ca, x0, y0, x0, y0, max_length, min_length);
  unsigned char val_after = ca.get(offset);
  ASSERT_FALSE(val_before == val_after);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
