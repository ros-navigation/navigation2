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

#ifndef NAV2_WORLD_MODEL__WORLD_MODEL_CLIENT_HPP_
#define NAV2_WORLD_MODEL__WORLD_MODEL_CLIENT_HPP_

#include "nav2_tasks/service_client.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_msgs/srv/process_region.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace nav2_world_model
{

using nav2_tasks::ServiceClient;
using nav2_msgs::srv::GetCostmap;
using nav2_msgs::srv::ProcessRegion;
using nav2_msgs::msg::Costmap;
using nav2_msgs::msg::CostmapMetaData;
using geometry_msgs::msg::Point;

class WorldModelClient
{
public:

  WorldModelClient()
  : costmap_client_("GetCostmap"),
    free_space_client_("ConfirmFreeSpace"),
    clear_area_client_("ClearArea")
  {
  }

  Costmap getCostmap(/*const*/ CostmapMetaData & specs)
  {
    // CONTINUE HERE!
    return costmap_client_.invoke(std::make_shared<GetCostmap::Request>(specs)).get()->map;
  }

  bool confirmFreeSpace(
    const double /*width*/, const double /*height*/, const Point /*center*/)
  {
    // TODO(orduno)
    return false;
  }

  bool clearArea(
    const double /*width*/, const double /*height*/, const Point /*center*/)
  {
    // TODO(orduno)
    return false;
  }

private:
  // Service clients
  ServiceClient<GetCostmap> costmap_client_;
  ServiceClient<ProcessRegion> free_space_client_;
  ServiceClient<ProcessRegion> clear_area_client_;
};

}  // namespace nav2_world_model

#endif  // NAV2_WORLD_MODEL__WORLD_MODEL_CLIENT_HPP_
