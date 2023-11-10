// Copyright (c) 2008, 2013, Willow Garage, Inc.
// Copyright (c) 2023 Samsung R&D Institute Russia
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
// Author: Eitan Marder-Eppstein
//         David V. Lu!!
//         Alexey Merzlyakov

#ifndef NAV2_UTIL__OCC_GRID_UTILS_HPP_
#define NAV2_UTIL__OCC_GRID_UTILS_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace nav2_util
{

/**
 * @brief: Convert from world coordinates to map coordinates.
   Similar to Costmap2D::worldToMap() method but works directly with OccupancyGrid-s.
 * @param  map OccupancyGrid map on which to convert
 * @param  wx The x world coordinate
 * @param  wy The y world coordinate
 * @param  mx Will be set to the associated map x coordinate
 * @param  my Will be set to the associated map y coordinate
 * @return True if the conversion was successful (legal bounds) false otherwise
 */
inline bool worldToMap(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map,
  const double wx, const double wy, unsigned int & mx, unsigned int & my)
{
  const double origin_x = map->info.origin.position.x;
  const double origin_y = map->info.origin.position.y;
  const double resolution = map->info.resolution;
  const unsigned int size_x = map->info.width;
  const unsigned int size_y = map->info.height;

  if (wx < origin_x || wy < origin_y) {
    return false;
  }

  mx = static_cast<unsigned int>((wx - origin_x) / resolution);
  my = static_cast<unsigned int>((wy - origin_y) / resolution);
  if (mx >= size_x || my >= size_y) {
    return false;
  }

  return true;
}

/**
 * @brief  Convert from map coordinates to world coordinates
 * @param  mx The x map coordinate
 * @param  my The y map coordinate
 * @param  wx Will be set to the associated world x coordinate
 * @param  wy Will be set to the associated world y coordinate
 */
inline void mapToWorld(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map,
  const unsigned int mx, const unsigned int my, double & wx, double & wy)
{
  const double origin_x = map->info.origin.position.x;
  const double origin_y = map->info.origin.position.y;
  const double resolution = map->info.resolution;

  wx = origin_x + (mx + 0.5) * resolution;
  wy = origin_y + (my + 0.5) * resolution;
}

}  // namespace nav2_util

#endif  // NAV2_UTIL__OCC_GRID_UTILS_HPP_
