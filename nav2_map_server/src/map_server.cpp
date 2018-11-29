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

#include <libgen.h>
#include <string>
#include <memory>
#include <fstream>
#include "nav2_map_server/occ_grid_loader.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

namespace nav2_map_server
{

MapServer::MapServer(const std::string & node_name)
: Node(node_name), origin_(3)
{
  getInputParameters();

  // Create the proper map instance for the specified map type
  if (map_type_ == "occupancy") {
    map_loader_ = std::make_unique<OccGridLoader>(this, origin_, resolution_);
  } else {
    RCLCPP_ERROR(get_logger(), "Cannot load map %s of type %s",
      map_name_.c_str(), map_type_.c_str());
    throw std::runtime_error("Map type not supported");
  }

  // Load the map and provide access to other nodes via topic and service
  map_loader_->loadMapFromFile(map_name_);
  map_loader_->initServices();
}

MapServer::MapServer()
: MapServer("map_server")
{
}

void MapServer::getInputParameters()
{
  get_parameter_or_set("yaml_filename", yaml_filename_, std::string("map.yaml"));
  get_parameter_or_set("map_type_", map_type_, std::string("occupancy"));

  // Make sure that there's a valid file there
  std::ifstream fin(yaml_filename_.c_str());
  if (fin.fail()) {
    RCLCPP_ERROR(get_logger(), "Could not open '%s'", yaml_filename_.c_str());
    throw std::runtime_error("File not found");
  }

  // Open up the YAML file so we can grab the various map-specific fields,
  // The image name, the resolution and the origin
  YAML::Node doc = YAML::LoadFile(yaml_filename_);

  // Get the map name, the resolution, and the origin
  try {
    map_name_ = doc["image"].as<std::string>();
    if (map_name_.size() == 0) {
      RCLCPP_ERROR(get_logger(), "The image tag cannot be an empty string");
      throw std::runtime_error("The image tag cannot be an empty string.");
    }
    if (map_name_[0] != '/') {
      // dirname can modify what you pass it
      char * fname_copy = strdup(yaml_filename_.c_str());
      map_name_ = std::string(dirname(fname_copy)) + '/' + map_name_;
      free(fname_copy);
    }
  } catch (YAML::Exception) {
    RCLCPP_ERROR(get_logger(),
      "%s does not contain an image tag or it is invalid.", yaml_filename_.c_str());
    throw std::runtime_error("The map does not contain an image tag or it is invalid.");
  }

  try {
    resolution_ = doc["resolution"].as<double>();
  } catch (YAML::Exception) {
    RCLCPP_ERROR(get_logger(),
      "%s does not contain a resolution tag or it is invalid.", yaml_filename_.c_str());
    throw std::runtime_error("The map does not contain a resolution tag or it is invalid.");
  }

  try {
    origin_[0] = doc["origin"][0].as<double>();
    origin_[1] = doc["origin"][1].as<double>();
    origin_[2] = doc["origin"][2].as<double>();
  } catch (YAML::Exception) {
    RCLCPP_ERROR(get_logger(),
      "%s does not contain an origin tag or it is invalid.", yaml_filename_.c_str());
    throw std::runtime_error("The map does not contain an origin tag or it is invalid.");
  }

  RCLCPP_DEBUG(get_logger(), "map_name: %s", map_name_.c_str());
  RCLCPP_DEBUG(get_logger(), "resolution: %f", resolution_);
  RCLCPP_DEBUG(get_logger(), "origin[0]: %f", origin_[0]);
  RCLCPP_DEBUG(get_logger(), "origin[1]: %f", origin_[1]);
  RCLCPP_DEBUG(get_logger(), "origin[2]: %f", origin_[2]);
}

}  // namespace nav2_map_server
