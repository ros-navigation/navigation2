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

#ifndef NAV2_MAP_SERVER__MAP_LOADER_HPP_
#define NAV2_MAP_SERVER__MAP_LOADER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_map_server/map_reps/map_reps.hpp"

class MapLoader
{
public:
  BaseMapLoader * createMap(std::string mapType, rclcpp::Node::SharedPtr n, std::string filename);
  BaseMapLoader * createMap(std::string mapType);
};

#endif  // NAV2_MAP_SERVER__MAP_LOADER_HPP_
