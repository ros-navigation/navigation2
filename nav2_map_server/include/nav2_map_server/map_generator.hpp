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

#ifndef NAV2_MAP_SERVER__MAP_GENERATOR_HPP_
#define NAV2_MAP_SERVER__MAP_GENERATOR_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/srv/get_map.hpp"

namespace nav2_map_server
{

class MapGenerator : public rclcpp::Node
{
public:
  MapGenerator(const std::string & mapname, int threshold_occupied, int threshold_free);

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

  bool saved_map_;

private:
  std::string mapname_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr map_sub_;
  int threshold_occupied_;
  int threshold_free_;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_GENERATOR_HPP_
