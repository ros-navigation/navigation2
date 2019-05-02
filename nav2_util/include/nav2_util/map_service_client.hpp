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

#ifndef NAV2_UTIL__MAP_SERVICE_CLIENT_HPP_
#define NAV2_UTIL__MAP_SERVICE_CLIENT_HPP_

#include <memory>
#include <string>

#include "nav2_util/service_client.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"

namespace nav2_util
{

class MapServiceClient : public nav2_util::ServiceClient<nav_msgs::srv::GetMap>
{
public:
  explicit MapServiceClient(const std::string & parent_node_name)
  : nav2_util::ServiceClient<nav_msgs::srv::GetMap>("map", parent_node_name)
  {
  }

  explicit MapServiceClient(rclcpp::Node::SharedPtr node)
  : nav2_util::ServiceClient<nav_msgs::srv::GetMap>("map", node)
  {
  }

  bool getMap(nav_msgs::msg::OccupancyGrid & map)
  {
    auto request = std::make_shared<nav2_util::MapServiceClient::MapServiceRequest>();
    auto response = std::make_shared<nav2_util::MapServiceClient::MapServiceResponse>();

    bool rc = invoke(request, response);
    if (rc) {
      map = response->map;
    }

    return rc;
  }

  using MapServiceRequest = nav2_util::ServiceClient<nav_msgs::srv::GetMap>::RequestType;
  using MapServiceResponse = nav2_util::ServiceClient<nav_msgs::srv::GetMap>::ResponseType;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__MAP_SERVICE_CLIENT_HPP_
