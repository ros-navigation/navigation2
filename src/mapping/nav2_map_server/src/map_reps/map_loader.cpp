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

#include <string>

#include "nav2_map_server/map_loader.hpp"

namespace nav2_map_server
{

// Factory Class for loading new map types
BaseMapLoader * MapLoader::CreateMap(
  std::string map_type,
  rclcpp::Node::SharedPtr node, std::string file_name)
{
  if (map_type == "occupancy") {
    return new OccGridLoader(node, file_name);
  } else if (map_type == "gridmap") {
    return new OccGridLoader(node, file_name);  // TODO(bpwilcox): Substitute with GridMapLoader
  } else {
    fprintf(stderr, "[ERROR] [map_server]: Cannot Load Map of Type '%s'\n", map_type.c_str());
    exit(-1);
  }
}

BaseMapLoader * MapLoader::CreateMap(std::string map_type, rclcpp::Node::SharedPtr node)
{
  if (map_type == "occupancy") {
    return new OccGridLoader(node);
  } else if (map_type == "gridmap") {
    return new OccGridLoader(node);  // TODO(bpwilcox): Substitute with GridMapLoader
  } else {
    fprintf(stderr, "[ERROR] [map_server]: Cannot Load Map of Type '%s'\n", map_type.c_str());
    exit(-1);
  }
}

}  // namespace nav2_map_server
