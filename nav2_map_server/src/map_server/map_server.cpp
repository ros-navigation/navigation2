/* Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Copyright 2019 Rover Robotics
 * Copyright 2010 Brian Gerkey
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

#include "nav2_map_server/map_server.hpp"

#include <string>
#include <memory>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "yaml-cpp/yaml.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_map_server/map_io.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace nav2_map_server
{

MapServer::MapServer()
: nav2_util::LifecycleNode("map_server")
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare the node parameters
  declare_parameter("yaml_filename", rclcpp::PARAMETER_STRING);
  declare_parameter("topic_name", "map");
  declare_parameter("frame_id", "map");
}

MapServer::~MapServer()
{
}

nav2_util::CallbackReturn
MapServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Get the name of the YAML file to use
  std::string yaml_filename = get_parameter("yaml_filename").as_string();

  std::string topic_name = get_parameter("topic_name").as_string();
  frame_id_ = get_parameter("frame_id").as_string();

  // Shared pointer to LoadMap::Response is also should be initialized
  // in order to avoid null-pointer dereference
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> rsp =
    std::make_shared<nav2_msgs::srv::LoadMap::Response>();

  if (!loadMapResponseFromYaml(yaml_filename, rsp)) {
    throw std::runtime_error("Failed to load map yaml file: " + yaml_filename);
  }

  // Make name prefix for services
  const std::string service_prefix = get_name() + std::string("/");

  // Create a service that provides the occupancy grid
  occ_service_ = create_service<nav_msgs::srv::GetMap>(
    service_prefix + std::string(service_name_),
    std::bind(&MapServer::getMapCallback, this, _1, _2, _3));

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Create a service that loads the occupancy grid from a file
  load_map_service_ = create_service<nav2_msgs::srv::LoadMap>(
    service_prefix + std::string(load_map_service_name_),
    std::bind(&MapServer::loadMapCallback, this, _1, _2, _3));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Publish the map using the latched topic
  occ_pub_->on_activate();
  auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
  occ_pub_->publish(std::move(occ_grid));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  occ_pub_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  occ_pub_.reset();
  occ_service_.reset();
  load_map_service_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void MapServer::getMapCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav_msgs::srv::GetMap::Request>/*request*/,
  std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
{
  // if not in ACTIVE state, ignore request
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received GetMap request but not in ACTIVE state, ignoring!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling GetMap request");
  response->map = msg_;
}

void MapServer::loadMapCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  // if not in ACTIVE state, ignore request
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received LoadMap request but not in ACTIVE state, ignoring!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling LoadMap request");
  // Load from file
  if (loadMapResponseFromYaml(request->map_url, response)) {
    auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
    occ_pub_->publish(std::move(occ_grid));  // publish new map
  }
}

bool MapServer::loadMapResponseFromYaml(
  const std::string & yaml_file,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  switch (loadMapFromYaml(yaml_file, msg_)) {
    case MAP_DOES_NOT_EXIST:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
      return false;
    case INVALID_MAP_METADATA:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA;
      return false;
    case INVALID_MAP_DATA:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
      return false;
    case LOAD_MAP_SUCCESS:
      // Correcting msg_ header when it belongs to spiecific node
      updateMsgHeader();

      response->map = msg_;
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
  }

  return true;
}

void MapServer::updateMsgHeader()
{
  msg_.info.map_load_time = now();
  msg_.header.frame_id = frame_id_;
  msg_.header.stamp = now();
}

}  // namespace nav2_map_server
