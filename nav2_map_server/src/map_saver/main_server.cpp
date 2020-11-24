// Copyright (c) 2020 Samsung Research Russia
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
#include <stdexcept>
#include <string>

#include "nav2_map_server/map_saver.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("map_saver_server");
  auto service_node = std::make_shared<nav2_map_server::MapSaver>();
  rclcpp::spin(service_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
