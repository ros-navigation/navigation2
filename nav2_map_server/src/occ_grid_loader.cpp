/* Copyright 2019 Rover Robotics
 * Copyright 2018 Brian Gerkey
 * Copyright (c) 2008, Willow Garage, Inc.
 *
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

#include "nav2_map_server/occ_grid_loader.hpp"

#include <libgen.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "Magick++.h"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"
#include "nav2_util/geometry_utils.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

namespace nav2_map_server
{
using nav2_util::geometry_utils::orientationAroundZAxis;

OccGridLoader::OccGridLoader(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string & yaml_filename)
: node_(node), yaml_filename_(yaml_filename)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Creating");
}

OccGridLoader::~OccGridLoader() {RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Destroying");}

/// Get the given subnode value.
/// The only reason this function exists is to wrap the exceptions in slightly nicer error messages,
/// including the name of the failed key
/// @throw YAML::Exception
template<typename T>
T yaml_get_value(const YAML::Node & node, const std::string & key)
{
  try {
    return node[key].as<T>();
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}

OccGridLoader::LoadParameters OccGridLoader::load_map_yaml(const std::string & yaml_filename)
{
  YAML::Node doc = YAML::LoadFile(yaml_filename);
  LoadParameters loadParameters;

  auto image_file_name = yaml_get_value<std::string>(doc, "image");
  if (image_file_name.empty()) {
    throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
  }
  if (image_file_name[0] != '/') {
    // dirname takes a mutable char *, so we copy into a vector
    std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
    fname_copy.push_back('\0');
    image_file_name = std::string(dirname(fname_copy.data())) + '/' + image_file_name;
  }
  loadParameters.image_file_name = image_file_name;

  loadParameters.resolution = yaml_get_value<double>(doc, "resolution");
  loadParameters.origin = yaml_get_value<std::vector<double>>(doc, "origin");
  if (loadParameters.origin.size() != 3) {
    throw YAML::Exception(
            doc["origin"].Mark(), "value of the 'origin' tag should have 3 elements, not " +
            std::to_string(loadParameters.origin.size()));
  }

  loadParameters.free_thresh = yaml_get_value<double>(doc, "free_thresh");
  loadParameters.occupied_thresh = yaml_get_value<double>(doc, "occupied_thresh");

  auto map_mode_node = doc["mode"];
  if (!map_mode_node.IsDefined()) {
    loadParameters.mode = MapMode::Trinary;
  } else {
    loadParameters.mode = map_mode_from_string(map_mode_node.as<std::string>());
  }

  try {
    loadParameters.negate = yaml_get_value<int>(doc, "negate");
  } catch (YAML::Exception &) {
    loadParameters.negate = yaml_get_value<bool>(doc, "negate");
  }

  RCLCPP_DEBUG(node_->get_logger(), "resolution: %f", loadParameters.resolution);
  RCLCPP_DEBUG(node_->get_logger(), "origin[0]: %f", loadParameters.origin[0]);
  RCLCPP_DEBUG(node_->get_logger(), "origin[1]: %f", loadParameters.origin[1]);
  RCLCPP_DEBUG(node_->get_logger(), "origin[2]: %f", loadParameters.origin[2]);
  RCLCPP_DEBUG(node_->get_logger(), "free_thresh: %f", loadParameters.free_thresh);
  RCLCPP_DEBUG(node_->get_logger(), "occupied_thresh: %f", loadParameters.occupied_thresh);
  RCLCPP_DEBUG(node_->get_logger(), "mode: %s", map_mode_to_string(loadParameters.mode));
  RCLCPP_DEBUG(node_->get_logger(), "negate: %d", loadParameters.negate);

  return loadParameters;
}

bool OccGridLoader::loadMapFromYaml(
  std::string yaml_file,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  if (yaml_file.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "YAML file name is empty, can't load!");
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loading yaml file: %s", yaml_file.c_str());
  LoadParameters loadParameters;
  try {
    loadParameters = load_map_yaml(yaml_file);
  } catch (YAML::Exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed processing YAML file %s at position (%d:%d) for reason: %s",
      yaml_file.c_str(), e.mark.line, e.mark.column, e.what());
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA;
    return false;
  } catch (std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to parse map YAML loaded from file %s for reason: %s",
      yaml_file.c_str(), e.what());
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA;
    return false;
  }

  try {
    loadMapFromFile(loadParameters);
  } catch (std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to load image file %s for reason: %s",
      loadParameters.image_file_name.c_str(), e.what());
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
    return false;
  }
  return true;
}

nav2_util::CallbackReturn OccGridLoader::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Configuring");

  // initialize Occupancy Grid msg - needed by loadMapFromYaml
  msg_ = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  if (!loadMapFromYaml(yaml_filename_)) {
    throw std::runtime_error("Failed to load map yaml file: " + yaml_filename_);
  }

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

    // Create the load_map service callback handle
  auto load_map_callback = [this](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response) -> void {
      // // if not in ACTIVE state, ignore request
      if (node_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Received LoadMap request but not in ACTIVE state, ignoring!");
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Handling LoadMap request");
      // Load from file
      if (loadMapFromYaml(request->map_url, response)) {
        // response->map = *msg_;
        response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
        occ_pub_->publish(*msg_);  // publish new map
      }
    };

  // Create a service that loads the occupancy grid from a file
  load_map_service_ = node_->create_service<nav2_msgs::srv::LoadMap>(
    load_map_service_name_,
    load_map_callback);

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  occ_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn OccGridLoader::on_activate(const rclcpp_lifecycle::State & /*state*/)
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

nav2_util::CallbackReturn OccGridLoader::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Deactivating");

  occ_pub_->on_deactivate();
  timer_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn OccGridLoader::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Cleaning up");

  occ_pub_.reset();
  occ_service_.reset();
  msg_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

void OccGridLoader::loadMapFromFile(const LoadParameters & loadParameters)
{
  Magick::InitializeMagick(nullptr);
  nav_msgs::msg::OccupancyGrid msg;

  Magick::Image img(loadParameters.image_file_name);

  // Copy the image data into the map structure
  msg.info.width = img.size().width();
  msg.info.height = img.size().height();
  msg.info.resolution = loadParameters.resolution;
  msg.info.origin.position.x = loadParameters.origin[0];
  msg.info.origin.position.y = loadParameters.origin[1];
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation = orientationAroundZAxis(loadParameters.origin[2]);

  // Allocate space to hold the data
  msg.data.resize(msg.info.width * msg.info.height);

  // Copy pixel data into the map structure
  for (size_t y = 0; y < msg.info.height; y++) {
    for (size_t x = 0; x < msg.info.width; x++) {
      auto pixel = img.pixelColor(x, y);

      std::vector<Magick::Quantum> channels = {pixel.redQuantum(), pixel.greenQuantum(),
        pixel.blueQuantum()};
      if (loadParameters.mode == MapMode::Trinary && img.matte()) {
        // To preserve existing behavior, average in alpha with color channels in Trinary mode.
        // CAREFUL. alpha is inverted from what you might expect. High = transparent, low = opaque
        channels.push_back(MaxRGB - pixel.alphaQuantum());
      }
      double sum = 0;
      for (auto c : channels) {
        sum += c;
      }
      /// on a scale from 0.0 to 1.0 how bright is the pixel?
      double shade = Magick::ColorGray::scaleQuantumToDouble(sum / channels.size());

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels occupied. Otherwise, it's vice versa.
      /// on a scale from 0.0 to 1.0, how occupied is the map cell (before thresholding)?
      double occ = (loadParameters.negate ? shade : 1.0 - shade);

      int8_t map_cell;
      switch (loadParameters.mode) {
        case MapMode::Trinary:
          if (loadParameters.occupied_thresh < occ) {
            map_cell = 100;
          } else if (occ < loadParameters.free_thresh) {
            map_cell = 0;
          } else {
            map_cell = -1;
          }
          break;
        case MapMode::Scale:
          if (pixel.alphaQuantum() != OpaqueOpacity) {
            map_cell = -1;
          } else if (loadParameters.occupied_thresh < occ) {
            map_cell = 100;
          } else if (occ < loadParameters.free_thresh) {
            map_cell = 0;
          } else {
            map_cell = std::rint(
              (occ - loadParameters.free_thresh) /
              (loadParameters.occupied_thresh - loadParameters.free_thresh) * 100.0);
          }
          break;
        case MapMode::Raw: {
            double occ_percent = std::round(shade * 255);
            if (0 <= occ_percent && occ_percent <= 100) {
              map_cell = static_cast<int8_t>(occ_percent);
            } else {
              map_cell = -1;
            }
            break;
          }
        default:
          throw std::runtime_error("Invalid map mode");
      }
      msg.data[msg.info.width * (msg.info.height - y - 1) + x] = map_cell;
    }
  }

  msg.info.map_load_time = node_->now();
  msg.header.frame_id = frame_id_;
  msg.header.stamp = node_->now();

  RCLCPP_DEBUG(
    node_->get_logger(), "Read map %s: %d X %d map @ %.3lf m/cell",
    loadParameters.image_file_name.c_str(), msg.info.width, msg.info.height, msg.info.resolution);

  *msg_ = msg;
}

}  // namespace nav2_map_server
