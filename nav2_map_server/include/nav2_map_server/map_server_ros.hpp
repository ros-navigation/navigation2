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

#ifndef NAV2_MAP_SERVER__MAP_SERVER_ROS_HPP_
#define NAV2_MAP_SERVER__MAP_SERVER_ROS_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_map_server/map_factory.hpp"
#include "nav2_map_server/map_representations/map_reps.hpp"

namespace nav2_map_server
{

class MapServerROS
{
public:
  MapServerROS(const std::string & file_name, const std::string & map_type);

private:
  MapFactory * map_loader_;
  BaseMapServer * map_;

  // TODO(bpwilcox): Add converter for map representations

  rclcpp::Node::SharedPtr node_;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_SERVER_ROS_HPP_
