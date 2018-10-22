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
#include "nav2_map_server/map_representations/occ_grid_server.hpp"

#include <libgen.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <chrono>

#include "yaml-cpp/yaml.h"
#include "LinearMath/btQuaternion.h"
#include "SDL/SDL_image.h"

using namespace std::chrono_literals;

namespace nav2_map_server
{

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

template<typename T>
void operator>>(const YAML::Node & node, T & i)
{
  i = node.as<T>();
}

OccGridServer::OccGridServer(rclcpp::Node::SharedPtr node, std::string file_name)
: node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridServer: Load map info for map file: %s",
    file_name.c_str());
  LoadMapInfoFromFile(file_name);

  RCLCPP_INFO(node_->get_logger(), "OccGridServer: Loading Map: %s", map_name_.c_str());
  LoadMapFromFile(map_name_);

  ConnectROS();
  SetMap();
  PublishMap();
  RCLCPP_INFO(node_->get_logger(), "OccGridServer: Set up map request service and publisher.");
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
    doc["resolution"] >> res_;
  } catch (YAML::Exception) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"),
      "The map %s does not contain a resolution tag or it is invalid.", file_name.c_str());
    throw std::runtime_error("The map does not contain a resolution tag or it is invalid.");
  }

  try {
    doc["negate"] >> negate_;
  } catch (YAML::Exception) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"),
      "The map %s does not contain a negate tag or it is invalid.", file_name.c_str());
    throw std::runtime_error("The map does not contain a negate tag or it is invalid.");
  }

  try {
    doc["occupied_thresh"] >> occ_th_;
  } catch (YAML::Exception) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"),
      "The map %s does not contain an occupied_thresh tag or it is invalid.", file_name.c_str());
    throw std::runtime_error("The map does not contain an occupied_thresh tag or it is invalid.");
  }

  try {
    doc["free_thresh"] >> free_th_;
  } catch (YAML::Exception) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"),
      "The map %s does not contain a free_thresh tag or it is invalid.", file_name.c_str());
    throw std::runtime_error("The map does not contain a free_thresh tag or it is invalid.");
  }

  try {
    std::string modeS = "";
    doc["mode"] >> modeS;
    if (modeS == "trinary") {
      mode_ = TRINARY;
    } else if (modeS == "scale") {
      mode_ = SCALE;
    } else if (modeS == "raw") {
      mode_ = RAW;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("map_server"),
        "The map %s has invalid mode tag \"%s\".", file_name.c_str(), modeS.c_str());
      throw std::runtime_error("Invalid mode tag.");
    }
  } catch (YAML::Exception) {
    RCLCPP_DEBUG(rclcpp::get_logger("map_server"),
      "The map %s does not contain a mode tag or it is invalid.", file_name.c_str());
    mode_ = TRINARY;
  }

  try {
    doc["origin"][0] >> origin_[0];
    doc["origin"][1] >> origin_[1];
    doc["origin"][2] >> origin_[2];
  } catch (YAML::Exception) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"),
      "The map %s does not contain an origin tag or it is invalid.", file_name.c_str());
    throw std::runtime_error("The map does not contain an origin tag or it is invalid.");
  }

  try {
    doc["image"] >> map_name_;
    // TODO(bpwilcox): make this path-handling more robust
    if (map_name_.size() == 0) {
      RCLCPP_ERROR(rclcpp::get_logger("map_server"),
        "The image tag in map %s cannot be an empty string.", file_name.c_str());
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
      "The map %s does not contain an image tag or it is invalid.", file_name.c_str());
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

  // Copy the image data into the map structure
  map_msg_.info.width = img->w;
  map_msg_.info.height = img->h;
  map_msg_.info.resolution = res_;
  map_msg_.info.origin.position.x = *(origin_);
  map_msg_.info.origin.position.y = *(origin_ + 1);
  map_msg_.info.origin.position.z = 0.0;
  btQuaternion q;
  // setEulerZYX(yaw, pitch, roll)
  q.setEulerZYX(*(origin_ + 2), 0, 0);
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
  if (mode_ == TRINARY || !img->format->Amask) {
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
      occ = (255 - color_avg) / 255.0;

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

  RCLCPP_INFO(rclcpp::get_logger("map_server"), "Read map %s: %d X %d map @ %.3lf m/cell",
    map_name_.c_str(),
    map_msg_.info.width,
    map_msg_.info.height,
    map_msg_.info.resolution);
}

void OccGridServer::ConnectROS()
{
  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 1;
  custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  occ_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "occ_grid", custom_qos_profile);

  // Create a service callback handle
  auto handle_occ_callback = [this](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request>/*request*/,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response) -> void {
      RCLCPP_INFO(node_->get_logger(), "OccGridServer: handle_occ_callback");
      response->map = occ_resp_.map;
    };

  // Create a service that provides the occupancy grid
  occ_service_ = node_->create_service<nav_msgs::srv::GetMap>("occ_grid", handle_occ_callback);
}

void OccGridServer::SetMap()
{
  occ_resp_.map = map_msg_;
}

void OccGridServer::PublishMap()
{
  occ_pub_->publish(map_msg_);

  // For now, periodically publish the map so that the ros1 bridge will be sure the proxy the
  // message to rviz on the ROS1 side
  // TODO(mjeronimo): Remove this once we've got everything on the ROS2 side
  auto timer_callback = [this]() -> void {occ_pub_->publish(map_msg_);};
  timer_ = node_->create_wall_timer(2s, timer_callback);
}

}  // namespace nav2_map_server
