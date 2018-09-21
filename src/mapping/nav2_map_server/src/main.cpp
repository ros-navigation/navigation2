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
#include "nav2_map_server/map_server_ros.hpp"

#define USAGE    "\nUSAGE: map_server <map.yaml> <map_type>\n" \
  "  map.yaml: map description file\n" \
  "  map_type: the type of map to load (i.e. occupancy)\n"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3 && argc != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"), "%s", USAGE);
    return -1;
  }

  std::string file_name(argv[1]);
  std::string map_type = (argc == 2) ? "occupancy" : std::string(argv[2]);

  try {
    nav2_map_server::MapServerROS MapServer(file_name, map_type);
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"), "%s", e.what());
    return -1;
  }

  return 0;
}
