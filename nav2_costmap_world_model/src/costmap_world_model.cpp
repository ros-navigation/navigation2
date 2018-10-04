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
  // Create Costmap with static and inflation layer
  layered_costmap_ = new costmap_2d::LayeredCostmap("frame", false, false);
  addStaticLayer();
  addInflationLayer();
  // TODO(bpwilcox): add footprint to layered_costmap either manually or from nav2_robot
  layered_costmap_->updateMap(0, 0, 0);

  auto costmap_service_callback = [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<nav2_world_model_msgs::srv::GetCostmap::Request> request,
      const std::shared_ptr<nav2_world_model_msgs::srv::GetCostmap::Response> response)->void
  {
    RCLCPP_INFO(
        this->get_logger(), "CostmapWorldModel::CostmapWorldModel:Incoming costmap request");
    costmap_callback(request_header, request, response);
  };

  // Create a service that will use the callback function to handle requests.
  costmapServer_ = create_service<nav2_msgs::srv::GetCostmap>("GetCostmap",
      costmap_service_callback);
}

void CostmapWorldModel::costmap_callback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav2_world_model_msgs::srv::GetCostmap::Request>/*request*/,
    const std::shared_ptr<nav2_world_model_msgs::srv::GetCostmap::Response> response)
{
  costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
  rclcpp::Clock clock;

  response->map.metadata.size_x = costmap->getSizeInCellsX();
  response->map.metadata.size_y = costmap->getSizeInCellsY();
  response->map.metadata.resolution = costmap->getResolution();
  response->map.metadata.layer = "Master";
  response->map.metadata.map_load_time = clock.now();
  response->map.metadata.update_time = clock.now();

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, 0.0);  // set roll, pitch, yaw
  response->map.metadata.origin.position.x = costmap->getOriginX();
  response->map.metadata.origin.position.y = costmap->getOriginY();
  response->map.metadata.origin.position.z = 0.0;
  response->map.metadata.origin.orientation.x = quaternion.x();
  response->map.metadata.origin.orientation.y = quaternion.y();
  response->map.metadata.origin.orientation.z = quaternion.z();
  response->map.metadata.origin.orientation.w = quaternion.w();

  response->map.header.stamp = clock.now();
  response->map.header.frame_id = "map";
  unsigned char * data = costmap->getCharMap();
  std::vector<unsigned char> my_map(
      data, data + response->map.metadata.size_x * response->map.metadata.size_y);
  response->map.data = my_map;
}

void CostmapWorldModel::addStaticLayer()
{
  costmap_2d::StaticLayer * slayer = new costmap_2d::StaticLayer();
  layered_costmap_->addPlugin(std::shared_ptr<costmap_2d::Layer>(slayer));
  slayer->initialize(layered_costmap_, "static", tf_);
}

void CostmapWorldModel::addInflationLayer()
{
  costmap_2d::InflationLayer * ilayer = new costmap_2d::InflationLayer();
  layered_costmap_->addPlugin(std::shared_ptr<costmap_2d::Layer>(ilayer));
  ilayer->initialize(layered_costmap_, "inflation", tf_);

}

CostmapWorldModel::CostmapWorldModel()
  : CostmapWorldModel("WorldModel")
{
}

}  // namespace nav2_costmap_world_model
