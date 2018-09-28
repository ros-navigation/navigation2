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
#include "nav2_map_server/map_reps/occ_grid_server.hpp"

#include <libgen.h>
#include <LinearMath/btQuaternion.h>
#include <SDL/SDL_image.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

namespace nav2_map_server
{

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

template<typename T>
void operator>>(const YAML::Node & node, T & i)
{
  i = node.as<T>();
}

void OccGridServer::LoadMapInfoFromFile(const std::string & file_name)
{
  std::ifstream fin(file_name.c_str());
  if (fin.fail()) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"),
      "Map_server could not open %s", file_name.c_str());
    throw std::runtime_error("File path invalid");
  }

  YAML::Node doc = YAML::LoadFile(file_name);

  try {
    doc["resolution"] >> res;
  } catch (YAML::Exception) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"),
      "The map does not contain a resolution tag or it is invalid.");
    throw std::runtime_error("The map does not contain a resolution tag or it is invalid.");
  }

  try {
    doc["negate"] >> negate;
  } catch (YAML::Exception) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"),
      "The map does not contain a negate tag or it is invalid.");
    throw std::runtime_error("The map does not contain a negate tag or it is invalid.");
  }

  try {
    doc["occupied_thresh"] >> occ_th;
  } catch (YAML::Exception) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"),
      "The map does not contain an occupied_thresh tag or it is invalid.");
    throw std::runtime_error("The map does not contain an occupied_thresh tag or it is invalid.");
  }

  try {
    doc["free_thresh"] >> free_th;
  } catch (YAML::Exception) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"),
      "The map does not contain a free_thresh tag or it is invalid.");
    throw std::runtime_error("The map does not contain a free_thresh tag or it is invalid.");
  }

  try {
    std::string modeS = "";
    doc["mode"] >> modeS;
    if (modeS == "trinary") {
      mode = TRINARY;
    } else if (modeS == "scale") {
      mode = SCALE;
    } else if (modeS == "raw") {
      mode = RAW;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("map_server"),
        "Invalid mode tag \"%s\".", modeS.c_str());
      throw std::runtime_error("Invalid mode tag.");
    }
  } catch (YAML::Exception) {
    RCLCPP_DEBUG(rclcpp::get_logger("map_server"),
      "The map does not contain a mode tag or it is invalid.");
    mode = TRINARY;
  }

  try {
    doc["origin"][0] >> origin[0];
    doc["origin"][1] >> origin[1];
    doc["origin"][2] >> origin[2];
  } catch (YAML::Exception) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"),
      "The map does not contain an origin tag or it is invalid.");
    throw std::runtime_error("The map does not contain an origin tag or it is invalid.");
  }

  try {
    doc["image"] >> map_name_;
    // TODO(bpwilcox): make this path-handling more robust
    if (map_name_.size() == 0) {
      RCLCPP_ERROR(rclcpp::get_logger("map_server"),
        "The image tag cannot be an empty string.");
      throw std::runtime_error("The image tag cannot be an empty string.");
    }
    if (map_name_[0] != '/') {
      // dirname can modify what you pass it
      char * fname_copy = strdup(file_name.c_str());
      map_name_ = std::string(dirname(fname_copy)) + '/' + map_name_;
      free(fname_copy);
    }
  } catch (YAML::Exception) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"),
      "The map does not contain an image tag or it is invalid.");
    throw std::runtime_error("The map does not contain an image tag or it is invalid.");
  }
}

void OccGridServer::LoadMapFromFile(const std::string & map_name_)
{
  SDL_Surface * img;
  const char * name = map_name_.c_str();
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
  if (!(img = IMG_Load(name))) {
    std::string errmsg = std::string("failed to open image file \"") +
      std::string(name) + std::string("\": ") + IMG_GetError();
    RCLCPP_ERROR(rclcpp::get_logger("map_server"), "%s", errmsg.c_str());

    throw std::runtime_error(errmsg);
  }

// TODO:
  map_msg_.header.frame_id = "map";
  map_msg_.info.map_load_time = node_->now();

  // Copy the image data into the map structure
  map_msg_.info.width = img->w;
  map_msg_.info.height = img->h;
  map_msg_.info.resolution = res;
  map_msg_.info.origin.position.x = *(origin);
  map_msg_.info.origin.position.y = *(origin + 1);
  map_msg_.info.origin.position.z = 0.0;
  btQuaternion q;
  // setEulerZYX(yaw, pitch, roll)
  q.setEulerZYX(*(origin + 2), 0, 0);
  map_msg_.info.origin.orientation.x = q.x();
  map_msg_.info.origin.orientation.y = q.y();
  map_msg_.info.origin.orientation.z = q.z();
  map_msg_.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  map_msg_.data.resize(map_msg_.info.width * map_msg_.info.height);

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
  for (j = 0; j < map_msg_.info.height; j++) {
    for (i = 0; i < map_msg_.info.width; i++) {
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
        map_msg_.data[MAP_IDX(map_msg_.info.width, i, map_msg_.info.height - j - 1)] = value;
        continue;
      }

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels free.  Otherwise, it's vice versa.
      occ = (255 - color_avg) / 255.0;

      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      if (occ > occ_th) {
        value = +100;
      } else if (occ < free_th) {
        value = 0;
      } else if (mode == TRINARY || alpha < 1.0) {
        value = -1;
      } else {
        double ratio = (occ - free_th) / (occ_th - free_th);
        value = 99 * ratio;
      }

      map_msg_.data[MAP_IDX(map_msg_.info.width, i, map_msg_.info.height - j - 1)] = value;
    }
  }

  SDL_FreeSurface(img);
}

void OccGridServer::ConnectROS()
{
  // Create a publisher
  // TODO(bpwilcox): publish a latched topic
  occ_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "occ_grid", rmw_qos_profile_default);

  // Create a service callback handle
  auto handle_occ_callback = [this](
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response) -> void {
      OccMapCallback(request_header, request, response);
    };

  // Create a service
  occ_service_ = node_->create_service<nav_msgs::srv::GetMap>("occ_grid", handle_occ_callback);
}

void OccGridServer::SetMap()
{
  occ_resp_.map = map_msg_;
}

void OccGridServer::PublishMap()
{
  occ_pub_->publish(map_msg_);
}

OccGridServer::OccGridServer(rclcpp::Node::SharedPtr node, std::string file_name)
: node_(node)
{
  // Set up //

  RCLCPP_INFO(node_->get_logger(), "Load map info");
  LoadMapInfoFromFile(file_name);

  RCLCPP_INFO(node_->get_logger(), "Load Map: %s", map_name_.c_str());
  LoadMapFromFile(map_name_);

  ConnectROS();

  RCLCPP_INFO(node_->get_logger(), "Set up Service");
  SetMap();

  RCLCPP_INFO(node_->get_logger(), "Set up Publisher");
  PublishMap();

  RCLCPP_INFO(node_->get_logger(), "Success!");
}

void OccGridServer::OccMapCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
  const std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
{
  (void)request_header;
  (void)req;
//  occ_resp_.map = map_msg_;
  res->map = occ_resp_.map;

  RCLCPP_INFO(node_->get_logger(), "OccGridServer::OccMapCallback");
  occ_pub_->publish(map_msg_);
}

}  // namespace nav2_map_server
