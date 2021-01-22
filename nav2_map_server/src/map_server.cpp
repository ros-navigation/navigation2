// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_map_server/map_server.hpp"

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_map_server/occ_grid_loader.hpp"
#include "nav2_util/node_utils.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

namespace nav2_map_server
{

MapServer::MapServer()
: nav2_util::LifecycleNode("map_server")
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare the node parameters
  declare_parameter("yaml_filename", rclcpp::ParameterValue(std::string("map.yaml")));
  declare_parameter("topic_name", rclcpp::ParameterValue(std::string("map")));
  declare_parameter("frame_id", rclcpp::ParameterValue(std::string("map")));
}

MapServer::~MapServer()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
MapServer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Get the name of the YAML file to use
  std::string yaml_filename;
  std::string topic_name;
  std::string frame_id;
  get_parameter("yaml_filename", yaml_filename);
  get_parameter("topic_name", topic_name);
  get_parameter("frame_id", frame_id);

  // Make sure that there's a valid file there and open it up
  std::ifstream fin(yaml_filename.c_str());
  if (fin.fail()) {
    throw std::runtime_error("Could not open '" + yaml_filename + "': file not found");
  }

  // The YAML document from which to get the conversion parameters
  YAML::Node doc = YAML::LoadFile(yaml_filename);

  // Get the map type so that we can create the correct map loader
  std::string map_type;
  try {
    map_type = doc["map_type"].as<std::string>();
  } catch (YAML::Exception &) {
    // Default to occupancy grid if not specified in the YAML file
    map_type = "occupancy";
  }

  // Create the correct map loader for the specified map type
  if (map_type == "occupancy") {
    map_loader_ = std::make_unique<OccGridLoader>(shared_from_this(), yaml_filename, topic_name, frame_id);
  } else {
    std::string msg = "Cannot load unknown map type: '" + map_type + "'";
    throw std::runtime_error(msg);
  }

  map_loader_->on_configure(state);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");
  map_loader_->on_activate(state);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  map_loader_->on_deactivate(state);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  map_loader_->on_cleanup(state);
  map_loader_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace nav2_map_server
