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
#include "nav2_map_server/map_factory.hpp"
#include "nav2_map_server/map_server_ros.hpp"

namespace nav2_map_server
{

MapServerROS::MapServerROS(const std::string & file_name, const std::string & map_type)
{
  node_ = rclcpp::Node::make_shared("map_server");

  try {
    map_loader_ = new MapFactory();
    RCLCPP_INFO(node_->get_logger(), "Loading map %s of type '%s'", file_name.c_str(),
      map_type.c_str());
    map_ = map_loader_->CreateMap(map_type, node_, file_name);
    rclcpp::spin(node_);
  } catch (std::runtime_error e) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot load map %s of type %s because: %s.",
      file_name.c_str(), map_type.c_str(), e.what());
  }
}

}  // namespace nav2_map_server
