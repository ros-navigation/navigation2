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
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "nav2_map_server/map_factory.hpp"

void print_usage()
{
  printf("Usage: map_server [options]\n");
  printf("Options:\n");
  printf("  -h                Display usage information\n");
  printf("  -f                YAML file for map to load\n");
  printf("  -t map_type       Type of the map server to run\n");
}

int main(int argc, char ** argv)
{
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // Get the YAML filename (required)
  char * option = rcutils_cli_get_option(argv, argv + argc, "-f");

  if (option == nullptr) {
    print_usage();
    return -1;
  }

  std::string file_name = std::string(option);

  // Get the map type (optional, defaults to occupancy grid)
  std::string map_type("occupancy");
  option = rcutils_cli_get_option(argv, argv + argc, "-t");

  if (option != nullptr) {
    map_type = std::string(option);
  }

  // Now that we have the parameters, get started
  rclcpp::init(argc, argv);

  // Create the node that will be used by the map server. Creating the node first
  // allows any sub-objects, such as the map_server to use the Node::SharedPtr in their
  // constructors, which wouldn't be the case if the MapServer itself was directly a Node
  rclcpp::Node::SharedPtr map_server_node = std::make_shared<rclcpp::Node>("map_server_node");

  // Create the map server that uses this node
  auto map_server = nav2_map_server::MapFactory::createMap(map_server_node, map_type, file_name);

  rclcpp::spin(map_server_node);
  rclcpp::shutdown();

  return 0;
}
