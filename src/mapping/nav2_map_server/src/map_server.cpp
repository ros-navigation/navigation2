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
#include "nav2_map_server/map_loader.hpp"
#include <string>

#define USAGE    "\nUSAGE: map_server <map.yaml> <map_type>\n" \
  "  map.yaml: map description file\n" \
  "  map_type: the type of map to load (i.e. occupancy)\n"


MappingServerROS::MappingServerROS(const std::string & fname, const std::string & map_type)
{
  n = rclcpp::Node::make_shared("map_server");

  try {
    m = new MapLoader();

    RCLCPP_INFO(n->get_logger(), "Loading Map of Type '%s'", map_type.c_str());
    MyMap = m->createMap(map_type, n, fname);
    rclcpp::spin(n);
  } catch (std::runtime_error e) {
    RCLCPP_ERROR(n->get_logger(), "Cannot load map");
    exit(-1);
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3 && argc != 2) {
    fprintf(stderr, "[ERROR] [map_server]: %s", USAGE);
    exit(-1);
  }

  std::string fname(argv[1]);
  std::string map_type = (argc == 2) ? "occupancy" : std::string(argv[2]);

  try {
    MappingServerROS Server(fname, map_type);
  } catch (std::runtime_error & e) {
    fprintf(stderr, "[ERROR] [map_server]: map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
