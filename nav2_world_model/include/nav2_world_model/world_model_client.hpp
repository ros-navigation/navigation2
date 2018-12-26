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
#include "nav2_world_model/costmap_service_client.hpp"
#include "nav2_world_model/clear_area_service_client.hpp"
#include "nav2_world_model/free_space_service_client.hpp"
#include "nav2_msgs/msg/costmap.hpp"

namespace nav2_world_model
{

using nav2_msgs::msg::Costmap;
using nav2_msgs::msg::CostmapMetaData;
using geometry_msgs::msg::Point;

class WorldModelClient
{
public:

  WorldModelClient()
  {
  }

  Costmap getCostmap(const CostmapServiceRequest & specs)
  {
    auto request = std::make_shared<CostmapServiceRequest>(specs);
    return costmap_client_.invoke(request).get()->map;
  }

  bool confirmFreeSpace(const FreeSpaceServiceRequest & specs)
  {
    auto request = std::make_shared<FreeSpaceServiceRequest>(specs);
    return free_space_client_.invoke(request).get()->was_successful;
  }

  bool clearArea(const ClearAreaServiceRequest & specs)
  {
    auto request = std::make_shared<ClearAreaServiceRequest>(specs);
    return free_space_client_.invoke(request).get()->was_successful;
  }

private:
  // Service clients
  CostmapServiceClient costmap_client_;
  FreeSpaceServiceClient free_space_client_;
  ClearAreaServiceClient clear_area_client_;
};

}  // namespace nav2_world_model

#endif  // NAV2_WORLD_MODEL__WORLD_MODEL_CLIENT_HPP_
