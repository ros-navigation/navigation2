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

#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include "nav2_costmap_world_model/costmap_world_model.hpp"

using std::vector;
using std::string;

namespace nav2_costmap_world_model
{

CostmapWorldModel::CostmapWorldModel(const string & name)
: Node(name + "_Node")
{
  costmap_ = std::make_unique<nav2_util::Costmap>(this);

  auto costmap_service_callback = [this](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav2_world_model_msgs::srv::GetCostmap::Request> request,
    const std::shared_ptr<nav2_world_model_msgs::srv::GetCostmap::Response> response) -> void
    {
      RCLCPP_INFO(
        this->get_logger(), "CostmapWorldModel::CostmapWorldModel:Incoming costmap request");
      response->map = costmap_->getCostmap(request->specs);
    };

  // Create a service that will use the callback function to handle requests.
  costmapServer_ = create_service<nav2_world_model_msgs::srv::GetCostmap>(name + "_GetCostmap",
      costmap_service_callback);

  // Get the current map from the map server
  //
  // TODO(mjeronimo): Instead of using a service call, the map server should push any
  // map updates using a latched topic. Unfortunately, no latched topics yet in ROS2
  map_client_.waitForService(std::chrono::seconds(2));

  auto request = std::make_shared<nav2_tasks::MapServiceClient::MapServiceRequest>();
  auto response = map_client_.invoke(request);

  costmap_->setStaticMap(response->map);
}

CostmapWorldModel::CostmapWorldModel()
: CostmapWorldModel("WorldModel")
{
}

}  // namespace nav2_costmap_world_model
