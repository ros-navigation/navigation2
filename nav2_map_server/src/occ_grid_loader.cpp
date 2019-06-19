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

// This file contains helper functions for loading images as maps.
// Author: Brian Gerkey

#include "nav2_map_server/occ_grid_loader.hpp"

#include <libgen.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"
#include "SDL/SDL_image.h"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

namespace nav2_map_server
{

OccGridLoader::OccGridLoader(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::string & yaml_filename)
: node_(node), yaml_filename_(yaml_filename)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Creating");
}

OccGridLoader::~OccGridLoader()
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Destroying");
}

nav2_util::CallbackReturn
OccGridLoader::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Configuring");

  msg_ = std::make_unique<nav_msgs::msg::OccupancyGrid>();

  // The YAML document from which to get the conversion parameters
  YAML::Node doc = YAML::LoadFile(yaml_filename_);
  LoadParameters loadParameters;

  // Get the name of the map file
  std::string map_filename;
  try {
    map_filename = doc["image"].as<std::string>();
    if (map_filename.size() == 0) {
      throw std::runtime_error("The image tag cannot be an empty string");
    }
    if (map_filename[0] != '/') {
      // dirname can modify what you pass it
      char * fname_copy = strdup(yaml_filename_.c_str());
      map_filename = std::string(dirname(fname_copy)) + '/' + map_filename;
      free(fname_copy);
    }
  } catch (YAML::Exception &) {
    throw std::runtime_error("The map does not contain an image tag or it is invalid");
  }

  try {
    loadParameters.resolution = doc["resolution"].as<double>();
  } catch (YAML::Exception &) {
    throw std::runtime_error("The map does not contain a resolution tag or it is invalid");
  }

  try {
    loadParameters.origin[0] = doc["origin"][0].as<double>();
    loadParameters.origin[1] = doc["origin"][1].as<double>();
    loadParameters.origin[2] = doc["origin"][2].as<double>();
  } catch (YAML::Exception &) {
    throw std::runtime_error("The map does not contain an origin tag or it is invalid");
  }

  try {
    loadParameters.free_thresh = doc["free_thresh"].as<double>();
  } catch (YAML::Exception &) {
    throw std::runtime_error("The map does not contain a free_thresh tag or it is invalid");
  }

  try {
    loadParameters.occupied_thresh = doc["occupied_thresh"].as<double>();
  } catch (YAML::Exception &) {
    throw std::runtime_error("The map does not contain an occupied_thresh tag or it is invalid");
  }

  std::string mode_str;
  try {
    mode_str = doc["mode"].as<std::string>();

    // Convert the string version of the mode name to one of the enumeration values
    if (mode_str == "trinary") {
      loadParameters.mode = TRINARY;
    } else if (mode_str == "scale") {
      loadParameters.mode = SCALE;
    } else if (mode_str == "raw") {
      loadParameters.mode = RAW;
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "Mode parameter not recognized: '%s', using default value (trinary)",
        mode_str.c_str());
      loadParameters.mode = TRINARY;
    }
  } catch (YAML::Exception &) {
    RCLCPP_WARN(node_->get_logger(), "Mode parameter not set, using default value (trinary)");
    loadParameters.mode = TRINARY;
  }

  try {
    loadParameters.negate = doc["negate"].as<int>();
  } catch (YAML::Exception &) {
    throw std::runtime_error("The map does not contain a negate tag or it is invalid");
  }

  RCLCPP_DEBUG(node_->get_logger(), "resolution: %f", loadParameters.resolution);
  RCLCPP_DEBUG(node_->get_logger(), "origin[0]: %f", loadParameters.origin[0]);
  RCLCPP_DEBUG(node_->get_logger(), "origin[1]: %f", loadParameters.origin[1]);
  RCLCPP_DEBUG(node_->get_logger(), "origin[2]: %f", loadParameters.origin[2]);
  RCLCPP_DEBUG(node_->get_logger(), "free_thresh: %f", loadParameters.free_thresh);
  RCLCPP_DEBUG(node_->get_logger(), "occupied_thresh: %f", loadParameters.occupied_thresh);
  RCLCPP_DEBUG(node_->get_logger(), "mode_str: %s", mode_str.c_str());
  RCLCPP_DEBUG(node_->get_logger(), "mode: %d", loadParameters.mode);
  RCLCPP_DEBUG(node_->get_logger(), "negate: %d", loadParameters.negate);

  loadMapFromFile(map_filename, &loadParameters);

  // Create a service callback handle
  auto handle_occ_callback = [this](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request>/*request*/,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response) -> void {
      RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Handling map request");
      response->map = *msg_;
    };

  // Create a service that provides the occupancy grid
  occ_service_ = node_->create_service<nav_msgs::srv::GetMap>(service_name_, handle_occ_callback);

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  occ_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
OccGridLoader::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Activating");

  // Publish the map using the latched topic
  occ_pub_->on_activate();
  occ_pub_->publish(*msg_);

  // due to timing / discovery issues, need to republish map
  auto timer_callback = [this]() -> void {occ_pub_->publish(*msg_);};
  timer_ = node_->create_wall_timer(2s, timer_callback);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
OccGridLoader::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Deactivating");

  occ_pub_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
OccGridLoader::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Cleaning up");

  occ_pub_.reset();
  occ_service_.reset();
  msg_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

void
OccGridLoader::loadMapFromFile(const std::string & map_name, LoadParameters * loadParameters)
{
  RCLCPP_DEBUG(node_->get_logger(), "OccGridLoader: loadMapFromFile");

  // Load the image using SDL.  If we get NULL back, the image load failed.
  SDL_Surface * img;
  if (!(img = IMG_Load(map_name.c_str()))) {
    std::string errmsg = std::string("failed to open image file \"") +
      map_name + std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  msg_->info.width = img->w;
  msg_->info.height = img->h;
  msg_->info.resolution = loadParameters->resolution;
  msg_->info.origin.position.x = loadParameters->origin[0];
  msg_->info.origin.position.y = loadParameters->origin[1];
  msg_->info.origin.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, loadParameters->origin[2]);
  msg_->info.origin.orientation.x = q.x();
  msg_->info.origin.orientation.y = q.y();
  msg_->info.origin.orientation.z = q.z();
  msg_->info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  msg_->data.resize(msg_->info.width * msg_->info.height);

  // Get values that we'll need to iterate through the pixels
  int rowstride = img->pitch;
  int n_channels = img->format->BytesPerPixel;

  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  int avg_channels;
  if (loadParameters->mode == TRINARY || !img->format->Amask) {
    avg_channels = n_channels;
  } else {
    avg_channels = n_channels - 1;
  }

  // Copy pixel data into the map structure
  unsigned char * pixels = (unsigned char *)(img->pixels);
  int color_sum;
  for (unsigned int j = 0; j < msg_->info.height; j++) {
    for (unsigned int i = 0; i < msg_->info.width; i++) {
      // Compute mean of RGB for this pixel
      unsigned char * p = pixels + j * rowstride + i * n_channels;
      color_sum = 0;
      for (int k = 0; k < avg_channels; k++) {
        color_sum += *(p + (k));
      }
      double color_avg = color_sum / static_cast<double>(avg_channels);

      int alpha;
      if (n_channels == 1) {
        alpha = 1;
      } else {
        alpha = *(p + n_channels - 1);
      }

      if (loadParameters->negate) {
        color_avg = 255 - color_avg;
      }

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

      unsigned char value;
      if (loadParameters->mode == RAW) {
        value = color_avg;
        msg_->data[MAP_IDX(msg_->info.width, i, msg_->info.height - j - 1)] = value;
        continue;
      }

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels free.  Otherwise, it's vice versa.
      double occ = (255 - color_avg) / 255.0;

      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      if (occ > loadParameters->occupied_thresh) {
        value = +100;
      } else if (occ < loadParameters->free_thresh) {
        value = 0;
      } else if (loadParameters->mode == TRINARY || alpha < 1.0) {
        value = -1;
      } else {
        double ratio = (occ - loadParameters->free_thresh) /
          (loadParameters->occupied_thresh - loadParameters->free_thresh);
        value = 99 * ratio;
      }

      msg_->data[MAP_IDX(msg_->info.width, i, msg_->info.height - j - 1)] = value;
    }
  }

  SDL_FreeSurface(img);

  msg_->info.map_load_time = node_->now();
  msg_->header.frame_id = frame_id_;
  msg_->header.stamp = node_->now();

  RCLCPP_DEBUG(node_->get_logger(), "Read map %s: %d X %d map @ %.3lf m/cell",
    map_name.c_str(),
    msg_->info.width,
    msg_->info.height,
    msg_->info.resolution);
}

}  // namespace nav2_map_server
