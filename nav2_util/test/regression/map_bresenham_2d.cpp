// Copyright (c) 2022 Samsung R&D Institute Russia
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the <ORGANIZATION> nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Alexey Merzlyakov

#include <string.h>

#include <gtest/gtest.h>

#include "nav2_util/raytrace_line_2d.hpp"

// MapAction - is a functor class used to cover raytraceLine algorithm.
// It contains char map inside, which is an abstract one and not related
// to any concrete representation (like Costmap2D or OccupancyGrid).
class MapAction
{
public:
  explicit MapAction(
    char * map, unsigned int size, char mark_val = 100)
  : map_(map), size_(size), mark_val_(mark_val)
  {
  }

  inline void operator()(unsigned int off)
  {
    ASSERT_TRUE(off < size_);
    map_[off] = mark_val_;
  }

  inline unsigned int get(unsigned int off)
  {
    return map_[off];
  }

private:
  char * map_;
  unsigned int size_;
  char mark_val_;
};

class MapTest
{
public:
  MapTest(
    unsigned int size_x, unsigned int size_y,
    char default_val = 0)
  : size_x_(size_x), size_y_(size_y)
  {
    map_ = new char[size_x * size_y];
    memset(map_, default_val, size_x * size_y);
  }

  char * getMap()
  {
    return map_;
  }

  unsigned int getSize()
  {
    return size_x_ * size_y_;
  }

  void raytraceLine(
    MapAction ma, unsigned int x0, unsigned int y0, unsigned int x1,
    unsigned int y1,
    unsigned int max_length = UINT_MAX, unsigned int min_length = 0)
  {
    nav2_util::raytraceLine(ma, x0, y0, x1, y1, size_x_, max_length, min_length);
  }

private:
  char * map_;
  unsigned int size_x_, size_y_;
};

TEST(map_2d, bresenham2DBoundariesCheck)
{
  const unsigned int sz_x = 60;
  const unsigned int sz_y = 60;
  const unsigned int max_length = 60;
  const unsigned int min_length = 6;
  MapTest mt(sz_x, sz_y);
  MapAction ma(mt.getMap(), mt.getSize());

  // Initial point - some assymetrically standing point in order to cover most corner cases
  const unsigned int x0 = 2;
  const unsigned int y0 = 4;
  // (x1, y1) point will move
  unsigned int x1, y1;

  // Running on (x, 0) edge
  y1 = 0;
  for (x1 = 0; x1 < sz_x; x1++) {
    mt.raytraceLine(ma, x0, y0, x1, y1, max_length, min_length);
  }

  // Running on (x, sz_y) edge
  y1 = sz_y - 1;
  for (x1 = 0; x1 < sz_x; x1++) {
    mt.raytraceLine(ma, x0, y0, x1, y1, max_length, min_length);
  }

  // Running on (0, y) edge
  x1 = 0;
  for (y1 = 0; y1 < sz_y; y1++) {
    mt.raytraceLine(ma, x0, y0, x1, y1, max_length, min_length);
  }

  // Running on (sz_x, y) edge
  x1 = sz_x - 1;
  for (y1 = 0; y1 < sz_y; y1++) {
    mt.raytraceLine(ma, x0, y0, x1, y1, max_length, min_length);
  }
}

TEST(map_2d, bresenham2DSamePoint)
{
  const unsigned int sz_x = 60;
  const unsigned int sz_y = 60;
  const unsigned int max_length = 60;
  const unsigned int min_length = 0;
  MapTest mt(sz_x, sz_y, 0.1);
  MapAction ma(mt.getMap(), mt.getSize());

  // Initial point
  const double x0 = 2;
  const double y0 = 4;

  unsigned int offset = y0 * sz_x + x0;
  char val_before = ma.get(offset);
  // Same point to check
  mt.raytraceLine(ma, x0, y0, x0, y0, max_length, min_length);
  char val_after = ma.get(offset);
  ASSERT_FALSE(val_before == val_after);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
