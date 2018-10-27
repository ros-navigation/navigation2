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

/*
 * This file contains helper functions for loading images as maps.
 *
 * Author: Brian Gerkey
 */
#include "nav2_map_server/occ_grid_loader.hpp"

#include <string>
#include <stdexcept>
#include <chrono>

#include "LinearMath/btQuaternion.h"
#include "SDL/SDL_image.h"

using namespace std::chrono_literals;

namespace nav2_map_server
{

const std::string OccGridLoader::frame_id_ = "map";

OccGridLoader::OccGridLoader(rclcpp::Node * node)
: node_(node)
{
  loadParameters();
  loadMapFromFile(map_name_);
}

void OccGridLoader::loadParameters()
{
  std::string mode_str;

  // Get this node's default parameter values, using defaults if not supplied in the YAML file
  node_->get_parameter_or_set("resolution", res_, 0.050000);
  node_->get_parameter_or_set("negate", negate_, 0);
  node_->get_parameter_or_set("occupied_thresh", occ_th_, 0.65);
  node_->get_parameter_or_set("free_thresh", free_th_, 0.196);
  node_->get_parameter_or_set("mode", mode_str, std::string("trinary"));
  node_->get_parameter_or_set("image", map_name_, std::string("test_map.pgm"));
  node_->get_parameter_or_set("origin", origin_, std::vector<double>({-15.400000, -12.200000, 0.000000}));

  // Convert the string version of the mode name to one of the enumeration values
  if (mode_str == "trinary") {
    mode_ = TRINARY;
  } else if (mode_str == "scale") {
    mode_ = SCALE;
  } else if (mode_str == "raw") {
    mode_ = RAW;
  } else {
    RCLCPP_WARN(node_->get_logger(),
      "Mode parameter not recognized: '%s', using default value (trinary)",
      mode_str.c_str());
    mode_ = TRINARY;
  }
}

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

void OccGridLoader::loadMapFromFile(const std::string & map_name_)
{
  unsigned char * p;
  unsigned char value;
  unsigned int i, j;
  int k;

  // Load the image using SDL.  If we get NULL back, the image load failed.
  SDL_Surface * img;
  const char * name = map_name_.c_str();
  if (!(img = IMG_Load(name))) {
    std::string errmsg = std::string("failed to open image file \"") +
      std::string(name) + std::string("\": ") + IMG_GetError();
    RCLCPP_ERROR(rclcpp::get_logger("map_server"), "%s", errmsg.c_str());

    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  map_msg_.info.width = img->w;
  map_msg_.info.height = img->h;
  map_msg_.info.resolution = res_;
  map_msg_.info.origin.position.x = origin_[0];
  map_msg_.info.origin.position.y = origin_[1];
  map_msg_.info.origin.position.z = 0.0;
  btQuaternion q;
  // setEulerZYX(yaw, pitch, roll)
  q.setEulerZYX(origin_[2], 0, 0);
  map_msg_.info.origin.orientation.x = q.x();
  map_msg_.info.origin.orientation.y = q.y();
  map_msg_.info.origin.orientation.z = q.z();
  map_msg_.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  map_msg_.data.resize(map_msg_.info.width * map_msg_.info.height);

  // Get values that we'll need to iterate through the pixels
  int rowstride = img->pitch;
  int n_channels = img->format->BytesPerPixel;

  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  int avg_channels;
  if (mode_ == TRINARY || !img->format->Amask) {
    avg_channels = n_channels;
  } else {
    avg_channels = n_channels - 1;
  }

  // Copy pixel data into the map structure
  unsigned char * pixels = (unsigned char *)(img->pixels);
  int color_sum;
  for (j = 0; j < map_msg_.info.height; j++) {
    for (i = 0; i < map_msg_.info.width; i++) {
      // Compute mean of RGB for this pixel
      p = pixels + j * rowstride + i * n_channels;
      color_sum = 0;
      for (k = 0; k < avg_channels; k++) {
        color_sum += *(p + (k));
      }
      double color_avg = color_sum / static_cast<double>(avg_channels);

      int alpha;
      if (n_channels == 1) {
        alpha = 1;
      } else {
        alpha = *(p + n_channels - 1);
      }
      if (negate_) {
        color_avg = 255 - color_avg;
      }
      if (mode_ == RAW) {
        value = color_avg;
        map_msg_.data[MAP_IDX(map_msg_.info.width, i, map_msg_.info.height - j - 1)] = value;
        continue;
      }

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels free.  Otherwise, it's vice versa.
      double occ = (255 - color_avg) / 255.0;

      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      if (occ > occ_th_) {
        value = +100;
      } else if (occ < free_th_) {
        value = 0;
      } else if (mode_ == TRINARY || alpha < 1.0) {
        value = -1;
      } else {
        double ratio = (occ - free_th_) / (occ_th_ - free_th_);
        value = 99 * ratio;
      }

      map_msg_.data[MAP_IDX(map_msg_.info.width, i, map_msg_.info.height - j - 1)] = value;
    }
  }

  SDL_FreeSurface(img);

  map_msg_.info.map_load_time = node_->now();
  map_msg_.header.frame_id = frame_id_;
  map_msg_.header.stamp = node_->now();

  RCLCPP_DEBUG(node_->get_logger(), "Read map %s: %d X %d map @ %.3lf m/cell",
    map_name_.c_str(),
    map_msg_.info.width,
    map_msg_.info.height,
    map_msg_.info.resolution);
}

nav_msgs::msg::OccupancyGrid
OccGridLoader::getOccupancyGrid()
{
  return map_msg_;
}

}  // namespace nav2_map_server
