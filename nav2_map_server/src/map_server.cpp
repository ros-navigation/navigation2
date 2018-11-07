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

#include <string>
#include <memory>
#include <chrono>
#include "nav2_map_server/occ_grid_loader.hpp"

using namespace std::chrono_literals;

namespace nav2_map_server
{

MapServer::MapServer(const std::string & node_name)
: Node(node_name)
{
  // The params file can specify the map to load and its type
  get_parameter_or_set("map_name", map_name_, std::string("map.pgm"));
  get_parameter_or_set("map_type", map_type_, std::string("occupancy"));

  // Each instance of the map server will load one map/type, so the
  // map server creates the proper instance for the given type
  if (map_type_ == "occupancy") {
    map_loader_ = std::make_unique<OccGridLoader>(this);
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

}  // namespace nav2_map_server
