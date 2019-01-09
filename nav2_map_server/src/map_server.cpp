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
#include <stdexcept>
#include "nav2_map_server/occ_grid_loader.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

namespace nav2_map_server
{

MapServer::MapServer(const std::string & node_name)
: Node(node_name)
{
  // Get the MAP YAML file, which includes the image filename and the map type
  getParameters();

  // Create the proper map loader for the specified map type
  if (map_type_ == "occupancy") {
    map_loader_ = std::make_unique<OccGridLoader>(this, doc_);
  } else {
    std::string msg = "Cannot load unknown map type: '" + map_type_ + "'";
    throw std::runtime_error(msg);
  }

  // Load the map and provide access via topic and service interfaces
  map_loader_->loadMapFromFile(map_filename_);
  map_loader_->startServices();
}

MapServer::MapServer()
: MapServer("map_server")
{
}

void MapServer::getParameters()
{
  get_parameter_or_set("yaml_filename", yaml_filename_, std::string("map.yaml"));

  // Make sure that there's a valid file there and open it up
  std::ifstream fin(yaml_filename_.c_str());
  if (fin.fail()) {
    throw std::runtime_error("Could not open '" + yaml_filename_ + "': file not found");
  }

  doc_ = YAML::LoadFile(yaml_filename_);

  // Get the name of the map file
  try {
    map_filename_ = doc_["image"].as<std::string>();
    if (map_filename_.size() == 0) {
      throw std::runtime_error("The image tag cannot be an empty string");
    }
    if (map_filename_[0] != '/') {
      // dirname can modify what you pass it
      char * fname_copy = strdup(yaml_filename_.c_str());
      map_filename_ = std::string(dirname(fname_copy)) + '/' + map_filename_;
      free(fname_copy);
    }
  } catch (YAML::Exception) {
    std::string msg = "'" + yaml_filename_ + "' does not contain an image tag or it is invalid";
    throw std::runtime_error(msg);
  }

  // Get the map type so that we can create the correct map loader
  try {
    map_type_ = doc_["map_type"].as<std::string>();
  } catch (YAML::Exception) {
    // Default to occupancy grid if not specified in the YAML file
    map_type_ = "occupancy";
  }
}

}  // namespace nav2_map_server
