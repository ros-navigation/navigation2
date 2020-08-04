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
#include <yaml-cpp/yaml.h>

#include "boost/filesystem.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_map_server/map_io.hpp"
#include "nav2_map_server_3D/map_io_3D.hpp"
#include "nav2_msgs/msg/pcd2.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace nav2_map_server {

MapServer::MapServer()
: nav2_util::LifecycleNode("map_server")
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare the node parameters
  declare_parameter("yaml_filename");
  declare_parameter("topic_name", "map");
  declare_parameter("frame_id", "map");

  enable_pcd_ = false, enable_image_ = false;
}

MapServer::~MapServer()
{
}

void MapServer::CheckForFunctionalitiesToEnable(const std::string &yaml_filename)
{

  if (yaml_filename.empty()) {
    throw std::runtime_error("yaml file either not provided or is empty """);
  }

  YAML::Node doc = YAML::LoadFile(yaml_filename);
  std::string input_filename;

  // Check for image files
  input_filename = yaml_get_value<std::string>(doc, "image");

  //  check for file extension
  boost::filesystem::path p(input_filename);
  if (p.empty()) {
    throw YAML::Exception(doc["image"].Mark(), "The 'image' tag is empty.");
  }

  if (p.extension() == ".bmp" || p.extension() == ".pgm" || p.extension() == ".png") {
    if (enable_pcd_){
      throw std::runtime_error("The 'image' tag is trying to change "
        "server configuration from 3D to 2D which is invalid");
    }
    enable_image_ = true;
    enable_pcd_ = false;
  } else if (p.extension() == ".pcd") {
    if (enable_image_){
      throw std::runtime_error("The 'image' tag is trying to change "
        "server configuration from 2D to 3D which is invalid");
    }
    enable_pcd_ = true;
    enable_image_ = false;
  } else{
    throw std::runtime_error("The 'image' tag in yaml_file has not "
      "a valid extension , it should be one of .bmp/.pgm/.png/.pcd");
  }
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
  MapServer::CheckForFunctionalitiesToEnable(yaml_filename);

  std::cout << enable_image_ << " " << enable_pcd_ << std::endl;
  if (enable_pcd_) {

    std::shared_ptr<nav2_msgs::srv::LoadMap3D::Response> rsp =
      std::make_shared<nav2_msgs::srv::LoadMap3D::Response>();

    if (!loadMapResponseFromYaml(yaml_filename, rsp)) {
      throw std::runtime_error("Failed to load map yaml file: " + yaml_filename);
    }

    // Make name prefix for services
    const std::string service_prefix = get_name() + std::string("/");

    // Create a service that provides the PointCloud2
    auto get_map_callback_lambda = [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<nav2_msgs::srv::GetMap3D::Request> request,
        std::shared_ptr<nav2_msgs::srv::GetMap3D::Response> response) {
      getMapCallback(request_header, request, response);
    };

    pcd_service_ = create_service<nav2_msgs::srv::GetMap3D>(
      service_prefix + std::string(service_name_),
      get_map_callback_lambda);

    // Create a publisher using the QoS settings to emulate a ROS1 latched topic
    pcd_pub_ = create_publisher<nav2_msgs::msg::PCD2>(
      topic_name,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    // Create a service that loads the PointCloud2 from a file
    auto load_map_callback_lambda = [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<nav2_msgs::srv::LoadMap3D::Request> request,
        std::shared_ptr<nav2_msgs::srv::LoadMap3D::Response> response) {
      loadMapCallback(request_header, request, response);
    };

    pcd_load_map_service_ = create_service<nav2_msgs::srv::LoadMap3D>(
      service_prefix + std::string(load_map_service_name_), load_map_callback_lambda);

    std::cout << "enabled: pcd" << std::endl;
  } else if (enable_image_) {

    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> rsp = std::make_shared<nav2_msgs::srv::LoadMap::Response>();
    if (!loadMapResponseFromYaml(yaml_filename, rsp)) {
      throw std::runtime_error("Failed to load map yaml file: " + yaml_filename);
    }

    // Make name prefix for services
    const std::string service_prefix = get_name() + std::string("/");

    // Create a service that provides the occupancy grid
    auto get_map_callback_lambda = [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
        std::shared_ptr<nav_msgs::srv::GetMap::Response> response) {
      getMapCallback(request_header, request, response);
    };

    occ_service_ = create_service<nav_msgs::srv::GetMap>(
      service_prefix + std::string(service_name_), get_map_callback_lambda);

    // Create a publisher using the QoS settings to emulate a ROS1 latched topic
    occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      topic_name,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    // Create a service that loads the occupancy grid from a file
    auto load_map_callback_lambda = [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
        std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response) {
      loadMapCallback(request_header, request, response);
    };

    load_map_service_ = create_service<nav2_msgs::srv::LoadMap>(
      service_prefix + std::string(load_map_service_name_), load_map_callback_lambda);
    std::cout << "enabled: image" << std::endl;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Publish the map(pcd if enabled) using the latched topic
  if (enable_pcd_) {
    pcd_pub_->on_activate();
    auto pcd_cloud_2 = std::make_unique<nav2_msgs::msg::PCD2>(pcd_msg_);
    pcd_pub_->publish(std::move(pcd_cloud_2));
  }

  // Publish the map(occ if enabled) using the latched topic
  if (enable_image_) {
    occ_pub_->on_activate();
    auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
    occ_pub_->publish(std::move(occ_grid));
  }

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (enable_pcd_) {
    pcd_pub_->on_deactivate();
  }

  if (enable_image_) {
    occ_pub_->on_deactivate();
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  if (enable_pcd_) {
    pcd_pub_.reset();
    pcd_service_.reset();
    pcd_load_map_service_.reset();
  }

  if (enable_image_) {
    occ_pub_.reset();
    occ_service_.reset();
    load_map_service_.reset();
  }

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

void MapServer::getMapCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::GetMap3D::Request> /*request*/,
  std::shared_ptr<nav2_msgs::srv::GetMap3D::Response> response)
{
  // if not in ACTIVE state, ignore request
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
        get_logger(),
        "Received GetMap request but not in ACTIVE state, ignoring!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling GetMap request");
  response->map = pcd_msg_.map;
  response->origin = pcd_msg_.origin;
  response->orientation = pcd_msg_.orientation;
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

void MapServer::loadMapCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::LoadMap3D::Request> request,
  std::shared_ptr<nav2_msgs::srv::LoadMap3D::Response> response)
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
    auto pcd_msg = std::make_unique<nav2_msgs::msg::PCD2>(pcd_msg_);
    pcd_pub_->publish(std::move(pcd_msg));  // publish new map
  }
}

bool MapServer::loadMapResponseFromYaml(
  const std::string &yaml_file,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  switch (loadMapFromYaml(yaml_file, msg_)) {
    case MAP_DOES_NOT_EXIST:response->result = nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
      return false;
    case INVALID_MAP_METADATA:response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA;
      return false;
    case INVALID_MAP_DATA:response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
      return false;
    case LOAD_MAP_SUCCESS:
      // Correcting msg_ header when it belongs to specific node
      updateMsgHeader();

      response->map = msg_;
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
  }

  return true;
}

bool MapServer::loadMapResponseFromYaml(
  const std::string &yaml_file,
  std::shared_ptr<nav2_msgs::srv::LoadMap3D::Response> response)
{
  switch (nav2_map_server_3D::loadMapFromYaml(yaml_file, pcd_msg_)) {
    case nav2_map_server_3D::MAP_DOES_NOT_EXIST :
      response->result = nav2_msgs::srv::LoadMap3D::Response::RESULT_MAP_DOES_NOT_EXIST;
      return false;
    case nav2_map_server_3D::INVALID_MAP_METADATA:
      response->result = nav2_msgs::srv::LoadMap3D::Response::RESULT_INVALID_MAP_METADATA;
      return false;
    case nav2_map_server_3D::INVALID_MAP_DATA:
      response->result = nav2_msgs::srv::LoadMap3D::Response::RESULT_INVALID_MAP_DATA;
      return false;
    case nav2_map_server_3D::LOAD_MAP_SUCCESS:response->map = pcd_msg_.map;
      response->origin = pcd_msg_.origin;
      response->orientation = pcd_msg_.orientation;
      response->result = nav2_msgs::srv::LoadMap3D::Response::RESULT_SUCCESS;
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
