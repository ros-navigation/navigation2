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
#include <mutex>

using namespace std::placeholders;

namespace nav2_map_server
{
MapSaver::MapSaver(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("map_saver", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare the node parameters
  declare_parameter("save_map_timeout", 2.0);
  declare_parameter("free_thresh_default", 0.25);
  declare_parameter("occupied_thresh_default", 0.65);
  declare_parameter("map_subscribe_transient_local", true);
}

MapSaver::~MapSaver()
{
}

nav2_util::CallbackReturn
MapSaver::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Make name prefix for services
  const std::string service_prefix = get_name() + std::string("/");

  save_map_timeout_ = std::make_shared<rclcpp::Duration>(
    rclcpp::Duration::from_seconds(get_parameter("save_map_timeout").as_double()));
  free_thresh_default_ = get_parameter("free_thresh_default").as_double();
  occupied_thresh_default_ = get_parameter("occupied_thresh_default").as_double();
  map_subscribe_transient_local_ = get_parameter("map_subscribe_transient_local").as_bool();

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

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapSaver::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapSaver::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  save_map_service_.reset();

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

    std::promise<nav_msgs::msg::OccupancyGrid::SharedPtr> prom;
    std::future<nav_msgs::msg::OccupancyGrid::SharedPtr> future_result = prom.get_future();
    // A callback function that receives map message from subscribed topic
    auto mapCallback = [&prom](
      const nav_msgs::msg::OccupancyGrid::SharedPtr msg) -> void {
        prom.set_value(msg);
      };

    rclcpp::QoS map_qos(10);  // initialize to default
    if (map_subscribe_transient_local_) {
      map_qos.transient_local();
      map_qos.reliable();
      map_qos.keep_last(1);
    }

    // Create new CallbackGroup for map_sub
    auto callback_group = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);

    auto option = rclcpp::SubscriptionOptions();
    option.callback_group = callback_group;
    auto map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_loc, map_qos, mapCallback, option);

    // Create SingleThreadedExecutor to spin map_sub in callback_group
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_callback_group(callback_group, get_node_base_interface());
    // Spin until map message received
    auto timeout = save_map_timeout_->to_chrono<std::chrono::nanoseconds>();
    auto status = executor.spin_until_future_complete(future_result, timeout);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to spin map subscription");
      return false;
    }
    // map_sub is no more needed
    map_sub.reset();
    // Map message received. Saving it to file
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg = future_result.get();
    if (saveMapToFile(*map_msg, save_parameters_loc)) {
      RCLCPP_INFO(get_logger(), "Map saved successfully");
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to save the map");
      return false;
    }
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to save the map: %s", e.what());
    return false;
  }

  return false;
}

}  // namespace nav2_map_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_map_server::MapSaver)
