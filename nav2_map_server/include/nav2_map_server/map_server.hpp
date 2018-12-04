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
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_map_server/map_loader.hpp"
#include "yaml-cpp/yaml.h"

namespace nav2_map_server
{

class MapServer : public rclcpp::Node
{
public:
  explicit MapServer(const std::string & node_name);
  MapServer();

private:
  void getParameters();

  // The map server has one node parameter, the YAML filename
  std::string yaml_filename_;

  // The YAML document from which to get the conversion parameters
  YAML::Node doc_;

  // The map type ("occupancy") from the YAML document which specifies
  // the kind of loader to create
  std::string map_type_;
  std::unique_ptr<MapLoader> map_loader_;

  // The map filename ("image") from the YAML document to pass to the map loader
  std::string map_filename_;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_SERVER_HPP_
