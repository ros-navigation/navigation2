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

MapServer::MapServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("map_server", "", options), map_available_(false)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare the node parameters
  declare_parameter("yaml_filename", rclcpp::PARAMETER_STRING);
  declare_parameter("topic_name", "map");
  declare_parameter("frame_id", "map");
  declare_parameter("grid_map_frame_id", "grid_map");
}

MapServer::~MapServer()
{
}

nav2_util::CallbackReturn
MapServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Get the name of the YAML file to use (can be empty if no initial map should be used)
  std::string yaml_filename = get_parameter("yaml_filename").as_string();
  std::string topic_name = get_parameter("topic_name").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  grid_map_frame_id_ = get_parameter("grid_map_frame_id").as_string();

  // only try to load map if parameter was set
  if (!yaml_filename.empty()) {
    // Shared pointer to LoadMap::Response is also should be initialized
    // in order to avoid null-pointer dereference
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> rsp =
      std::make_shared<nav2_msgs::srv::LoadMap::Response>();

    if (!loadMapResponseFromYaml(yaml_filename, rsp)) {
      throw std::runtime_error("Failed to load map yaml file: " + yaml_filename);
    }
  } else {
    RCLCPP_INFO(
      get_logger(),
      "yaml-filename parameter is empty, set map through '%s'-service",
      load_map_service_name_.c_str());
  }

  // Make name prefix for services
  const std::string service_prefix = get_name() + std::string("/");

  // Create a service that provides the occupancy grid
  occ_service_ = create_service<nav_msgs::srv::GetMap>(
    service_prefix + std::string(occ_map_service_name_),
    std::bind(&MapServer::getMapCallback, this, _1, _2, _3));

  // Create a service that provides the occupancy grid
  grid_map_service_ = create_service<grid_map_msgs::srv::GetGridMap>(
    service_prefix + std::string(grid_map_service_name_),
    std::bind(&MapServer::getGridMapCallback, this, _1, _2, _3));
  // TODO(ivrolan): implement GetOctomap

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
    std::string("grid_map_") + topic_name,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
    std::string("octomap_") + topic_name,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Create a service that loads the occupancy grid from a file
  load_map_service_ = create_service<nav2_msgs::srv::LoadMap>(
    service_prefix + std::string(load_map_service_name_),
    std::bind(&MapServer::loadMapCallback, this, _1, _2, _3));

  // create tf broadcaster
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Publish the map using the latched topic
  occ_pub_->on_activate();
  if (map_available_) {
    auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
    occ_pub_->publish(std::move(occ_grid));
  }

  // Publish the grid_map using the latched topic
  grid_map_pub_->on_activate();
  if (map_available_) {
    tf_static_broadcaster_->sendTransform(map_to_grid_map_t_);

    auto grid_map_to_pub = std::make_unique<grid_map_msgs::msg::GridMap>(msg_grid_map_);
    grid_map_pub_->publish(std::move(grid_map_to_pub));
  }

  octomap_pub_->on_activate();
  if (map_available_) {
    auto octomap_to_pub = std::make_unique<octomap_msgs::msg::Octomap>(msg_octomap_);
    octomap_pub_->publish(std::move(octomap_to_pub));
  }
  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  occ_pub_->on_deactivate();
  grid_map_pub_->on_deactivate();
  octomap_pub_->on_deactivate();
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
  grid_map_pub_.reset();
  octomap_pub_.reset();
  grid_map_service_.reset();
  load_map_service_.reset();
  map_available_ = false;
  msg_ = nav_msgs::msg::OccupancyGrid();
  msg_grid_map_ = grid_map_msgs::msg::GridMap();
  msg_octomap_ = octomap_msgs::msg::Octomap();

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

void MapServer::getGridMapCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request>/*request*/,
  std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response)
{
  // if not in ACTIVE state, ignore request
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received GetGridMap request but not in ACTIVE state, ignoring!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling GetGridMap request");
  response->map = msg_grid_map_;
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
    response->result = response->RESULT_UNDEFINED_FAILURE;
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling LoadMap request");
  // Load from file
  if (loadMapResponseFromYaml(request->map_url, response)) {
    auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
    occ_pub_->publish(std::move(occ_grid));  // publish new map

    tf_static_broadcaster_->sendTransform(map_to_grid_map_t_);

    auto grid_map_to_pub = std::make_unique<grid_map_msgs::msg::GridMap>(msg_grid_map_);
    grid_map_pub_->publish(std::move(grid_map_to_pub));  // publish new map
  }
}

bool MapServer::loadMapResponseFromYaml(
  const std::string & yaml_file,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  switch (loadMapFromYaml(yaml_file, msg_, msg_grid_map_, msg_octomap_)) {
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
      // Correcting msg_ header when it belongs to specific node
      updateMsgHeader();
      updateTransform();
      map_available_ = true;
      response->map = msg_;
      response->grid_map = msg_grid_map_;
      response->octomap = msg_octomap_;
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
  }

  return true;
}

void MapServer::updateMsgHeader()
{
  msg_.info.map_load_time = now();
  msg_.header.frame_id = frame_id_;
  msg_.header.stamp = now();

  msg_grid_map_.header.frame_id = grid_map_frame_id_;
  msg_grid_map_.header.stamp = now();

  msg_octomap_.header.frame_id = frame_id_;
  msg_octomap_.header.stamp = now();
}

void MapServer::updateTransform()
{
  map_to_grid_map_t_.header.stamp = this->get_clock()->now();
  map_to_grid_map_t_.header.frame_id = frame_id_;
  map_to_grid_map_t_.child_frame_id = grid_map_frame_id_;

  // we are going to use the transform to set the pos
  map_to_grid_map_t_.transform.translation.x = msg_.info.origin.position.x;
  map_to_grid_map_t_.transform.translation.y = msg_.info.origin.position.y;
  map_to_grid_map_t_.transform.translation.z = msg_.info.origin.position.z;

  // therefore, the grid_map will have its position changed
  //  position in grid_map_frame = pos_in_map_frame - pos_origin_map_frame
  msg_grid_map_.info.pose.position.x -= msg_.info.origin.position.x;
  msg_grid_map_.info.pose.position.y -= msg_.info.origin.position.y;
  msg_grid_map_.info.pose.position.z -= msg_.info.origin.position.z;

  // set the orient to the same as the 2d map
  map_to_grid_map_t_.transform.rotation.x = msg_.info.origin.orientation.x;
  map_to_grid_map_t_.transform.rotation.y = msg_.info.origin.orientation.y;
  map_to_grid_map_t_.transform.rotation.z = msg_.info.origin.orientation.z;
  map_to_grid_map_t_.transform.rotation.w = msg_.info.origin.orientation.w;
}

}  // namespace nav2_map_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_map_server::MapServer)
