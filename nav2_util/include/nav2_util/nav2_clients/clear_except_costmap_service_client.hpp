// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_UTIL__NAV2_CLIENTS__CLEAR_EXCEPT_COSTMAP_SERVICE_CLIENT_HPP_
#define NAV2_UTIL__NAV2_CLIENTS__CLEAR_EXCEPT_COSTMAP_SERVICE_CLIENT_HPP_

#include <string>
#include "nav2_util/service_client.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"

namespace nav2_util
{

class ClearExceptCostmapServiceClient
  : public ServiceClient<nav2_msgs::srv::ClearCostmapExceptRegion>
{
public:
  explicit ClearExceptCostmapServiceClient(const std::string & service_name)
  : ServiceClient<nav2_msgs::srv::ClearCostmapExceptRegion>(service_name)
  {
  }

  using clearExceptCostmapServiceRequest =
    ServiceClient<nav2_msgs::srv::ClearCostmapExceptRegion>::RequestType;
  using clearExceptCostmapServiceResponse =
    ServiceClient<nav2_msgs::srv::ClearCostmapExceptRegion>::ResponseType;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__NAV2_CLIENTS__CLEAR_EXCEPT_COSTMAP_SERVICE_CLIENT_HPP_
