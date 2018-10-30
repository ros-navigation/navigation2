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
  // TODO(mjeronimo): Currently, there is only one map loader. Going forward,
  // there will be multiple loaders, each with their own file to load. At that
  // point we'll have to figure out how to communicate which loaders and files
  // the map server should use.

  get_parameter_or_set("map_name", map_name_, std::string("map.pgm"));

  map_loader_ = std::make_unique<OccGridLoader>(this);
  map_loader_->loadMapFromFile(map_name_);
  map_loader_->initServices();
}

MapServer::MapServer()
: MapServer("map_server")
{
}

}  // namespace nav2_map_server
