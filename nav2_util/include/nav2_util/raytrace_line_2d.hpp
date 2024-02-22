// Copyright (c) 2008, 2013, Willow Garage, Inc.
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
//  * Neither the name of Willow Garage, Inc. nor the names of its
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
// Author: Eitan Marder-Eppstein
//         David V. Lu!!

#ifndef NAV2_UTIL__RAYTRACE_LINE_2D_HPP_
#define NAV2_UTIL__RAYTRACE_LINE_2D_HPP_

#include <limits.h>
#include <algorithm>
#include <cmath>

namespace nav2_util
{

/**
 * @brief get the sign of an int
 */
inline int sign(int x)
{
  return x > 0 ? 1.0 : -1.0;
}

/**
 * @brief  A 2D implementation of Bresenham's raytracing algorithm...
 * applies an action at each step
 */
template<class ActionType>
inline void bresenham2D(
  ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b,
  int offset_a, int offset_b, unsigned int offset,
  unsigned int max_length)
{
  unsigned int end = std::min(max_length, abs_da);
  for (unsigned int i = 0; i < end; ++i) {
    at(offset);
    offset += offset_a;
    error_b += abs_db;
    if ((unsigned int)error_b >= abs_da) {
      offset += offset_b;
      error_b -= abs_da;
    }
  }
  at(offset);
}

/**
 * @brief  Raytrace a line and apply some action at each step
 * @param  at The action to take... a functor
 * @param  x0 The starting x coordinate
 * @param  y0 The starting y coordinate
 * @param  x1 The ending x coordinate
 * @param  y1 The ending y coordinate
 * @param  step_x OX-step on map
 * @param  max_length The maximum desired length of the segment...
 * allows you to not go all the way to the endpoint
 * @param  min_length The minimum desired length of the segment
 */
template<class ActionType>
inline void raytraceLine(
  ActionType at, unsigned int x0, unsigned int y0, unsigned int x1,
  unsigned int y1, unsigned int step_x,
  unsigned int max_length = UINT_MAX, unsigned int min_length = 0)
{
  int dx_full = x1 - x0;
  int dy_full = y1 - y0;

  // we need to chose how much to scale our dominant dimension,
  // based on the maximum length of the line
  double dist = std::hypot(dx_full, dy_full);
  if (dist < min_length) {
    return;
  }

  unsigned int min_x0, min_y0;
  if (dist > 0.0) {
    // Adjust starting point and offset to start from min_length distance
    min_x0 = (unsigned int)(x0 + dx_full / dist * min_length);
    min_y0 = (unsigned int)(y0 + dy_full / dist * min_length);
  } else {
    // dist can be 0 if [x0, y0]==[x1, y1].
    // In this case only this cell should be processed.
    min_x0 = x0;
    min_y0 = y0;
  }
  unsigned int offset = min_y0 * step_x + min_x0;

  int dx = x1 - min_x0;
  int dy = y1 - min_y0;

  unsigned int abs_dx = abs(dx);
  unsigned int abs_dy = abs(dy);

  int offset_dx = sign(dx);
  int offset_dy = sign(dy) * step_x;

  double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);
  // if x is dominant
  if (abs_dx >= abs_dy) {
    int error_y = abs_dx / 2;

    bresenham2D(
      at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
    return;
  }

  // otherwise y is dominant
  int error_x = abs_dy / 2;

  bresenham2D(
    at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
}

}  // namespace nav2_util

#endif  // NAV2_UTIL__RAYTRACE_LINE_2D_HPP_
