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

#ifndef NAV2_MAP_SERVER__MAP_SERVER_HPP_
#define NAV2_MAP_SERVER__MAP_SERVER_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_map_server/map_loader.hpp"

namespace nav2_map_server
{

class MapServer : public rclcpp::Node
{
public:
  explicit MapServer(const std::string & node_name);
  MapServer();

private:
  // Only one map loader so far
  std::shared_ptr<MapLoader> map_loader_;

  // The name and type of the map to load
  std::string map_name_;
  std::string map_type_;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_SERVER_HPP_
