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
#include <vector>
#include <memory>
#include <stdexcept>

#include "LinearMath/btQuaternion.h"
#include "SDL/SDL_image.h"

using namespace std::chrono_literals;

namespace nav2_map_server
{

const char * OccGridLoader::frame_id_ = "map";
const char * OccGridLoader::topic_name_ = "occ_grid";
const char * OccGridLoader::service_name_ = "occ_grid";

OccGridLoader::OccGridLoader(rclcpp::Node * node, YAML::Node & doc)
: node_(node), doc_(doc), origin_(3)
{
  try {
    resolution_ = doc_["resolution"].as<double>();
  } catch (YAML::Exception) {
    throw std::runtime_error("The map does not contain a resolution tag or it is invalid");
  }

  try {
    origin_[0] = doc_["origin"][0].as<double>();
    origin_[1] = doc_["origin"][1].as<double>();
    origin_[2] = doc_["origin"][2].as<double>();
  } catch (YAML::Exception) {
    throw std::runtime_error("The map does not contain an origin tag or it is invalid");
  }

  try {
    free_thresh_ = doc_["free_thresh"].as<double>();
  } catch (YAML::Exception) {
    throw std::runtime_error("The map does not contain a free_thresh tag or it is invalid");
  }

  try {
    occupied_thresh_ = doc_["occupied_thresh"].as<double>();
  } catch (YAML::Exception) {
    throw std::runtime_error("The map does not contain an occupied_thresh tag or it is invalid");
  }

  std::string mode_str;
  try {
    mode_str = doc_["mode"].as<std::string>();

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
  } catch (YAML::Exception &) {
    RCLCPP_WARN(node_->get_logger(), "Mode parameter not set, using default value (trinary)");
    mode_ = TRINARY;
  }

  try {
    negate_ = doc_["negate"].as<int>();
  } catch (YAML::Exception) {
    throw std::runtime_error("The map does not contain a negate tag or it is invalid");
  }

  RCLCPP_DEBUG(node_->get_logger(), "resolution: %f", resolution_);
  RCLCPP_DEBUG(node_->get_logger(), "origin[0]: %f", origin_[0]);
  RCLCPP_DEBUG(node_->get_logger(), "origin[1]: %f", origin_[1]);
  RCLCPP_DEBUG(node_->get_logger(), "origin[2]: %f", origin_[2]);
  RCLCPP_DEBUG(node_->get_logger(), "free_thresh: %f", free_thresh_);
  RCLCPP_DEBUG(node_->get_logger(), "occupied_thresh: %f", occupied_thresh_);
  RCLCPP_DEBUG(node_->get_logger(), "mode_str: %s", mode_str.c_str());
  RCLCPP_DEBUG(node_->get_logger(), "mode: %d", mode_);
  RCLCPP_DEBUG(node_->get_logger(), "negate: %d", negate_);
}

void OccGridLoader::loadMapFromFile(const std::string & map_name)
{
  // Load the image using SDL.  If we get NULL back, the image load failed.
  SDL_Surface * img;
  if (!(img = IMG_Load(map_name.c_str()))) {
    std::string errmsg = std::string("failed to open image file \"") +
      map_name + std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  msg_.info.width = img->w;
  msg_.info.height = img->h;
  msg_.info.resolution = resolution_;
  msg_.info.origin.position.x = origin_[0];
  msg_.info.origin.position.y = origin_[1];
  msg_.info.origin.position.z = 0.0;
  btQuaternion q;
  // setEulerZYX(yaw, pitch, roll)
  q.setEulerZYX(origin_[2], 0, 0);
  msg_.info.origin.orientation.x = q.x();
  msg_.info.origin.orientation.y = q.y();
  msg_.info.origin.orientation.z = q.z();
  msg_.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  msg_.data.resize(msg_.info.width * msg_.info.height);

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
  for (unsigned int j = 0; j < msg_.info.height; j++) {
    for (unsigned int i = 0; i < msg_.info.width; i++) {
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

      if (negate_) {
        color_avg = 255 - color_avg;
      }

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

      unsigned char value;
      if (mode_ == RAW) {
        value = color_avg;
        msg_.data[MAP_IDX(msg_.info.width, i, msg_.info.height - j - 1)] = value;
        continue;
      }

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels free.  Otherwise, it's vice versa.
      double occ = (255 - color_avg) / 255.0;

      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      if (occ > occupied_thresh_) {
        value = +100;
      } else if (occ < free_thresh_) {
        value = 0;
      } else if (mode_ == TRINARY || alpha < 1.0) {
        value = -1;
      } else {
        double ratio = (occ - free_thresh_) / (occupied_thresh_ - free_thresh_);
        value = 99 * ratio;
      }

      msg_.data[MAP_IDX(msg_.info.width, i, msg_.info.height - j - 1)] = value;
    }
  }

  SDL_FreeSurface(img);

  msg_.info.map_load_time = node_->now();
  msg_.header.frame_id = frame_id_;
  msg_.header.stamp = node_->now();

  RCLCPP_DEBUG(node_->get_logger(), "Read map %s: %d X %d map @ %.3lf m/cell",
    map_name.c_str(),
    msg_.info.width,
    msg_.info.height,
    msg_.info.resolution);
}

void OccGridLoader::startServices()
{
  // Create a service callback handle
  auto handle_occ_callback = [this](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request>/*request*/,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response) -> void {
      RCLCPP_INFO(node_->get_logger(), "Handling map request");
      response->map = msg_;
    };

  // Create a service that provides the occupancy grid
  occ_service_ = node_->create_service<nav_msgs::srv::GetMap>(service_name_, handle_occ_callback);

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 1;
  custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  occ_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name_, custom_qos_profile);

  // Publish the map using the latched topic
  occ_pub_->publish(msg_);

  // TODO(mjeronimo): Remove the following once we've got everything on the ROS2 side
  //
  // Periodically publish the map so that the ros1 bridge will be sure the proxy the
  // message to rviz on the ROS1 side
  auto timer_callback = [this]() -> void {occ_pub_->publish(msg_);};
  timer_ = node_->create_wall_timer(2s, timer_callback);
}

}  // namespace nav2_map_server
