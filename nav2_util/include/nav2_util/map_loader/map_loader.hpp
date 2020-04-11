// Copyright (c) 2008, Willow Garage, Inc.
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
//  * Neither the name of the Willow Garage, Inc. nor the names of its
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

// Author: Brian Gerkey

#ifndef NAV2_UTIL__MAP_LOADER__MAP_LOADER_HPP_
#define NAV2_UTIL__MAP_LOADER__MAP_LOADER_HPP_

#include <string>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/map_loader/visibility_control.h"

/** Map mode
 *  Default: TRINARY -
 *      value >= occ_th - Occupied (100)
 *      value <= free_th - Free (0)
 *      otherwise - Unknown
 *  SCALE -
 *      alpha < 1.0 - Unknown
 *      value >= occ_th - Occupied (100)
 *      value <= free_th - Free (0)
 *      otherwise - f( (free_th, occ_th) ) = (0, 100)
 *          (linearly map in between values to (0,100)
 *  RAW -
 *      value = value
 */
enum MapMode {TRINARY, SCALE, RAW};

namespace map_loader
{

/** Read the image from file and fill out the resp object, for later
 * use when our services are requested.
 *
 * @param image_file_name The image file to read from
 * @param resolution The resolution of the map (gets stored in resp)
 * @param negate If true, then whiter pixels are occupied, and blacker
 *               pixels are free
 * @param occupancy_threshold Threshold above which pixels are occupied
 * @param free_threshold Threshold below which pixels are free
 * @param origin Triple specifying 2-D pose of lower-left corner of image
 * @param mode Map mode
 * @throws std::runtime_error If the image file can't be loaded
 * */

nav_msgs::msg::OccupancyGrid MAP_LOADER_PUBLIC loadMapFromFile(
  const std::string image_file_name, const double resolution, const bool negate,
  const double occupancy_threshold, const double free_threshold,
  const geometry_msgs::msg::Twist origin, const MapMode mode = TRINARY);
}  // namespace map_loader

#endif  // NAV2_UTIL__MAP_LOADER__MAP_LOADER_HPP_
