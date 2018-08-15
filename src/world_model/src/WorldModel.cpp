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

#include "world_model/WorldModel.hpp"

using std::vector;
using std::string;

WorldModel::WorldModel(const string& name) : Node (name)
{
  costmap_ = std::make_unique<Costmap>(this);

  auto costmap_service_callback = [this](
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request> request,
    const std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response) -> void
  {
    (void)request_header; // suppress unused variable warning
    RCLCPP_INFO(this->get_logger(), "WorldModel::WorldModel:Incoming costmap request");
    costmap_->getCostmap(request->specs, response->map);
  };

  // Create a service that will use the callback function to handle requests.
  costmapServer_ = create_service<nav2_msgs::srv::GetCostmap>(name, costmap_service_callback);
}
