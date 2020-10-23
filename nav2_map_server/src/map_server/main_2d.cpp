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

//
// Created by shivam on 10/3/20.
//

#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_map_server/map_server.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  std::string node_name("map_server");

  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav2_map_server::MapServer<nav_msgs::msg::OccupancyGrid>>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
  } catch (std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger(node_name), ex.what());
    RCLCPP_ERROR(rclcpp::get_logger(node_name), "Exiting");
    return -1;
  }
}
