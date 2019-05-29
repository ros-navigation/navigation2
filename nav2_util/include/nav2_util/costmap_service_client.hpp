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

#ifndef NAV2_UTIL__COSTMAP_SERVICE_CLIENT_HPP_
#define NAV2_UTIL__COSTMAP_SERVICE_CLIENT_HPP_

#include <string>

#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_util/service_client.hpp"

namespace nav2_util
{

class CostmapServiceClient : public ServiceClient<nav2_msgs::srv::GetCostmap>
{
public:
  explicit CostmapServiceClient(const std::string & parent_node_name)
  : ServiceClient<nav2_msgs::srv::GetCostmap>("GetCostmap", parent_node_name)
  {
  }

  explicit CostmapServiceClient(rclcpp::Node::SharedPtr & node)
  : ServiceClient<nav2_msgs::srv::GetCostmap>("GetCostmap", node)
  {
  }

  using CostmapServiceRequest =
    ServiceClient<nav2_msgs::srv::GetCostmap>::RequestType;
  using CostmapServiceResponse =
    ServiceClient<nav2_msgs::srv::GetCostmap>::ResponseType;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__COSTMAP_SERVICE_CLIENT_HPP_
