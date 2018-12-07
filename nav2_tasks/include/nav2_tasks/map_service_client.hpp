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

#ifndef NAV2_TASKS__MAP_SERVICE_CLIENT_HPP_
#define NAV2_TASKS__MAP_SERVICE_CLIENT_HPP_

#include "nav2_tasks/service_client.hpp"
#include "nav_msgs/srv/get_map.hpp"

namespace nav2_tasks
{

class MapServiceClient : public ServiceClient<nav_msgs::srv::GetMap>
{
public:
  MapServiceClient()
  : ServiceClient<nav_msgs::srv::GetMap>("map")
  {
  }

  using MapServiceRequest = ServiceClient<nav_msgs::srv::GetMap>::RequestType;
  using MapServiceResponse = ServiceClient<nav_msgs::srv::GetMap>::ResponseType;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__MAP_SERVICE_CLIENT_HPP_
