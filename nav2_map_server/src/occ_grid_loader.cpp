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

#include <stdexcept>

#include "yaml-cpp/yaml.h"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

namespace nav2_map_server
{

OccGridLoader::OccGridLoader(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string & yaml_filename,
  std::string & topic_name, std::string & frame_id)
: node_(node), yaml_filename_(yaml_filename), topic_name_(topic_name), frame_id_(frame_id)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Creating");
}

OccGridLoader::~OccGridLoader()
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Destroying");
}

bool OccGridLoader::loadMapResponseFromYaml(
  const std::string & yaml_file,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  if (yaml_file.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "YAML file name is empty, can't load!");
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loading yaml file: %s", yaml_file.c_str());
  LoadParameters load_parameters;
  try {
    load_parameters = loadMapYaml(yaml_file);
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
    loadMapFromFile(load_parameters, *msg_);
  } catch (std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to load image file %s for reason: %s",
      load_parameters.image_file_name.c_str(), e.what());
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
    return false;
  }

  // Correcting msg_ header when it belongs to spiecific node_
  updateMsgHeader();

  response->map = *msg_;
  response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;

  return true;
}

nav2_util::CallbackReturn OccGridLoader::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Configuring");

  // initialize Occupancy Grid msg - needed by loadMapResponseFromYaml()
  msg_ = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  // Shared pointer to LoadMap::Response is also should be initialized
  // in order to avoid null-pointer dereference
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> rsp =
    std::make_shared<nav2_msgs::srv::LoadMap::Response>();

  if (!loadMapResponseFromYaml(yaml_filename_, rsp)) {
    throw std::runtime_error("Failed to load map yaml file: " + yaml_filename_);
  }

  // Create GetMap service callback handle
  auto handle_occ_callback = [this](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request>/*request*/,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response) -> void {
      if (node_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Received GetMap request but not in ACTIVE state, ignoring!");
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Handling GetMap request");
      response->map = *msg_;
    };

  // Make name prefix for services
  std::string service_prefix = node_->get_name() + std::string("/");

  // Create a service that provides the occupancy grid
  occ_service_ = node_->create_service<nav_msgs::srv::GetMap>(
    service_prefix + std::string(service_name_),
    handle_occ_callback);

  // Create the load_map service callback handle
  auto load_map_callback = [this](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response) -> void {
      // if not in ACTIVE state, ignore request
      if (node_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Received LoadMap request but not in ACTIVE state, ignoring!");
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Handling LoadMap request");
      // Load from file
      if (loadMapResponseFromYaml(request->map_url, response)) {
        occ_pub_->publish(*msg_);  // publish new map
      }
    };

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  occ_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Create a service that loads the occupancy grid from a file
  load_map_service_ = node_->create_service<nav2_msgs::srv::LoadMap>(
    service_prefix + std::string(load_map_service_name_),
    load_map_callback);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn OccGridLoader::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Activating");

  // Publish the map using the latched topic
  occ_pub_->on_activate();
  occ_pub_->publish(*msg_);

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
  load_map_service_.reset();
  msg_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

void OccGridLoader::updateMsgHeader()
{
  msg_->info.map_load_time = node_->now();
  msg_->header.frame_id = frame_id_;
  msg_->header.stamp = node_->now();
}

}  // namespace nav2_map_server
