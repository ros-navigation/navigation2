/*
 * Copyright (c) 2020 Samsung R&D Institute Russia
 * Copyright 2019 Rover Robotics
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

#include "nav2_map_server/map_saver.hpp"

#include <string>
#include <memory>
#include <stdexcept>

namespace nav2_map_server
{
MapSaver::MapSaver()
: nav2_util::LifecycleNode("map_saver")
{
  RCLCPP_INFO(get_logger(), "Creating");

  save_map_timeout_ = std::make_shared<rclcpp::Duration>(
    std::chrono::milliseconds(declare_parameter("save_map_timeout", 2000)));

  free_thresh_default_ = declare_parameter("free_thresh_default", 25),
  occupied_thresh_default_ = declare_parameter("occupied_thresh_default", 65);

  map_listener_ = rclcpp::Node::make_shared("map_listener");
}

MapSaver::~MapSaver()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
MapSaver::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  // Create SaveMap service callback handle
  auto save_map_callback = [this](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav2_msgs::srv::SaveMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::SaveMap::Response> response) -> void {
      // Set input arguments and call saveMapTopicToFile()
      SaveParameters save_parameters;
      save_parameters.map_file_name = request->map_url;
      save_parameters.image_format = request->image_format;
      save_parameters.free_thresh = request->free_thresh;
      save_parameters.occupied_thresh = request->occupied_thresh;
      try {
        save_parameters.mode = map_mode_from_string(request->map_mode);
      } catch (std::invalid_argument &) {
        save_parameters.mode = MapMode::Trinary;
        RCLCPP_WARN(
          get_logger(), "Map mode parameter not recognized: '%s', using default value (trinary)",
          request->map_mode.c_str());
      }

      std::string map_topic = request->map_topic;

      response->result = saveMapTopicToFile(map_topic, save_parameters);
    };

  // Make name prefix for services
  std::string service_prefix = get_name() + std::string("/");

  // Create a service that saves the occupancy grid from map topic to a file
  save_map_service_ = create_service<nav2_msgs::srv::SaveMap>(
    service_prefix + save_map_service_name_,
    save_map_callback);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapSaver::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapSaver::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapSaver::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapSaver::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapSaver::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool MapSaver::saveMapTopicToFile(std::string & map_topic, SaveParameters & save_parameters)
{
  RCLCPP_INFO(
    get_logger(), "Saving map from %s topic to %s file",
    map_topic.c_str(), save_parameters.map_file_name.c_str());

  try {
    // Reset map receiving indicator
    got_map_msg_ = false;

    // Correct map_topic if necessary
    if (map_topic == "") {
      map_topic = "map";
      RCLCPP_WARN(
        get_logger(), "Map topic unspecified. Map messages will be read from %s topic",
        map_topic.c_str());
    }

    // Set default for MapSaver node thresholds parameters
    if (save_parameters.free_thresh == 0) {
      RCLCPP_WARN(
        get_logger(),
        "Free threshold unspecified. Setting it to default value: %i",
        free_thresh_default_);
      save_parameters.free_thresh = free_thresh_default_;
    }
    if (save_parameters.occupied_thresh == 0) {
      RCLCPP_WARN(
        get_logger(),
        "Occupied threshold unspecified. Setting it to default value: %i",
        occupied_thresh_default_);
      save_parameters.occupied_thresh = occupied_thresh_default_;
    }

    // Add new subscription for incoming map topic
    auto map_sub = map_listener_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&MapSaver::mapCallback, this, std::placeholders::_1));

    rclcpp::Time start_time = now();
    while (rclcpp::ok()) {
      rclcpp::spin_some(map_listener_);

      if ((now() - start_time) > *save_map_timeout_) {
        RCLCPP_ERROR(get_logger(), "Failed to save the map: timeout");
        return false;
      }

      if (got_map_msg_) {
        // Map message received. Saving it to file
        if (saveMapToFile(*msg_, save_parameters)) {
          RCLCPP_INFO(get_logger(), "Map saved successfully");
          return true;
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to save the map");
          return false;
        }
      }

      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to save the map: %s", e.what());
    return false;
  }

  RCLCPP_ERROR(get_logger(), "This situation should never appear");
  return false;
}

void MapSaver::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  msg_ = msg;
  got_map_msg_ = true;
}

}  // namespace nav2_map_server
