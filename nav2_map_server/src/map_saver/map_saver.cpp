/*
 * Copyright (c) 2020 Samsung Research Russia
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
#include <functional>

using namespace std::placeholders;

namespace nav2_map_server
{
MapSaver::MapSaver()
: nav2_util::LifecycleNode("map_saver", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");

  save_map_timeout_ = std::make_shared<rclcpp::Duration>(
    std::chrono::milliseconds(declare_parameter("save_map_timeout", 2000)));

  free_thresh_default_ = declare_parameter("free_thresh_default", 0.25),
  occupied_thresh_default_ = declare_parameter("occupied_thresh_default", 0.65);
  // false only of foxy for backwards compatibility
  map_subscribe_transient_local_ = declare_parameter("map_subscribe_transient_local", false);
}

MapSaver::~MapSaver()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
MapSaver::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Make name prefix for services
  const std::string service_prefix = get_name() + std::string("/");

  // Create a service that saves the occupancy grid from map topic to a file
  save_map_service_ = create_service<nav2_msgs::srv::SaveMap>(
    service_prefix + save_map_service_name_,
    std::bind(&MapSaver::saveMapCallback, this, _1, _2, _3));

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
MapSaver::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void MapSaver::saveMapCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::SaveMap::Request> request,
  std::shared_ptr<nav2_msgs::srv::SaveMap::Response> response)
{
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

  response->result = saveMapTopicToFile(request->map_topic, save_parameters);
}

bool MapSaver::saveMapTopicToFile(
  const std::string & map_topic,
  const SaveParameters & save_parameters)
{
  // Local copies of map_topic and save_parameters that could be changed
  std::string map_topic_loc = map_topic;
  SaveParameters save_parameters_loc = save_parameters;

  RCLCPP_INFO(
    get_logger(), "Saving map from \'%s\' topic to \'%s\' file",
    map_topic_loc.c_str(), save_parameters_loc.map_file_name.c_str());

  try {
    // Pointer to map message received in the subscription callback
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg = nullptr;

    // Correct map_topic_loc if necessary
    if (map_topic_loc == "") {
      map_topic_loc = "map";
      RCLCPP_WARN(
        get_logger(), "Map topic unspecified. Map messages will be read from \'%s\' topic",
        map_topic_loc.c_str());
    }

    // Set default for MapSaver node thresholds parameters
    if (save_parameters_loc.free_thresh == 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Free threshold unspecified. Setting it to default value: %f",
        free_thresh_default_);
      save_parameters_loc.free_thresh = free_thresh_default_;
    }
    if (save_parameters_loc.occupied_thresh == 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Occupied threshold unspecified. Setting it to default value: %f",
        occupied_thresh_default_);
      save_parameters_loc.occupied_thresh = occupied_thresh_default_;
    }

    // A callback function that receives map message from subscribed topic
    auto mapCallback = [&map_msg](
      const nav_msgs::msg::OccupancyGrid::SharedPtr msg) -> void {
        map_msg = msg;
      };

    // Add new subscription for incoming map topic.
    // Utilizing local rclcpp::Node (rclcpp_node_) from nav2_util::LifecycleNode
    // as a map listener.
    rclcpp::QoS map_qos = rclcpp::SystemDefaultsQoS();  // initialize to default
    if (map_subscribe_transient_local_) {
      map_qos = rclcpp::QoS(10);
      map_qos.transient_local();
      map_qos.reliable();
      map_qos.keep_last(1);
    }
    auto map_sub = rclcpp_node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_loc, map_qos, mapCallback);

    rclcpp::Time start_time = now();
    while (rclcpp::ok()) {
      if ((now() - start_time) > *save_map_timeout_) {
        RCLCPP_ERROR(get_logger(), "Failed to save the map: timeout");
        return false;
      }

      if (map_msg) {
        // Map message received. Saving it to file
        if (saveMapToFile(*map_msg, save_parameters_loc)) {
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

  return false;
}

}  // namespace nav2_map_server
