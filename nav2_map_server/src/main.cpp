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

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_map_server/map_factory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create the node that will be used by the map server. Creating the node first
  // allows the map_server to use the Node::SharedPtr in its constructor, which
  // wouldn't be possible if the MapServer was directly a Node
  rclcpp::Node::SharedPtr map_server_node = std::make_shared<rclcpp::Node>("map_server");

  // Create the map server that uses this node
  auto map_server = nav2_map_server::MapFactory::createMap(map_server_node);

  rclcpp::spin(map_server_node);
  rclcpp::shutdown();

  return 0;
}
