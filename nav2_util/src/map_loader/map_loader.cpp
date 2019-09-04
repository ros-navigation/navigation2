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

//  Author: Brian Gerkey

#include "nav2_util/map_loader/map_loader.hpp"
#include <stdlib.h>
#include <stdio.h>
// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>
#include <string>
#include <stdexcept>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace map_loader
{

nav_msgs::msg::OccupancyGrid loadMapFromFile(
  const std::string image_file_name, const double resolution, const bool negate,
  const double occupancy_threshold, const double free_threshold,
  const geometry_msgs::msg::Twist origin, const MapMode mode)
{
  SDL_Surface * img;

  unsigned char * pixels;
  unsigned char * p;
  unsigned char value;
  int rowstride, n_channels, avg_channels;
  unsigned int i, j;
  int k;
  double occ;
  int alpha;
  int color_sum;
  double color_avg;

  // Load the image using SDL.  If we get NULL back, the image load failed.
  if (!(img = IMG_Load(image_file_name.c_str()))) {
    std::string errmsg = std::string("failed to open image file \"") +
      image_file_name + std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = img->w;
  map.info.height = img->h;
  map.info.resolution = resolution;
  map.info.origin.position.x = origin.linear.x;
  map.info.origin.position.y = origin.linear.y;
  map.info.origin.position.z = origin.linear.z;

  tf2::Quaternion quaternion;
  // yaw, pitch and roll are rotations in z, y, x respectively
  quaternion.setRPY(origin.angular.x, origin.angular.y, origin.angular.z);
  map.info.origin.orientation = tf2::toMsg(quaternion);

  // Allocate space to hold the data
  map.data.resize(map.info.width * map.info.height);

  // Get values that we'll need to iterate through the pixels
  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;

  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  if (mode == TRINARY || !img->format->Amask) {
    avg_channels = n_channels;
  } else {
    avg_channels = n_channels - 1;
  }

  // Copy pixel data into the map structure
  pixels = (unsigned char *)(img->pixels);
  for (j = 0; j < map.info.height; j++) {
    for (i = 0; i < map.info.width; i++) {
      // Compute mean of RGB for this pixel
      p = pixels + j * rowstride + i * n_channels;
      color_sum = 0;
      for (k = 0; k < avg_channels; k++) {
        color_sum += *(p + (k));
      }
      color_avg = color_sum / static_cast<double>(avg_channels);

      if (n_channels == 1) {
        alpha = 1;
      } else {
        alpha = *(p + n_channels - 1);
      }
      if (negate) {
        color_avg = 255 - color_avg;
      }
      if (mode == RAW) {
        value = color_avg;
        map.data[MAP_IDX(map.info.width, i, map.info.height - j - 1)] = value;
        continue;
      }

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels occupied.  Otherwise, it's vice versa.
      occ = (255 - color_avg) / 255.0;

      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      if (occ > occupancy_threshold) {
        value = +100;
      } else if (occ < free_threshold) {
        value = 0;
      } else if (mode == TRINARY || alpha < 1.0) {
        value = -1;
      } else {
        double ratio = (occ - free_threshold) / (occupancy_threshold - free_threshold);
        value = 99 * ratio;
      }

      map.data[MAP_IDX(map.info.width, i, map.info.height - j - 1)] = value;
    }
  }
  SDL_FreeSurface(img);

  return map;
}

}  // namespace map_loader
