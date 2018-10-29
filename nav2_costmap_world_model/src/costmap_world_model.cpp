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
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav2_costmap_world_model/costmap_world_model.hpp"

using std::vector;
using std::string;

namespace nav2_costmap_world_model
{

CostmapWorldModel::CostmapWorldModel(const string & name)
: Node(name + "_Node")
{
  node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

  // Create layered costmap with static and inflation layer
  layered_costmap_ = new nav2_costmap_2d::LayeredCostmap("frame", false, false);
  addLayer<nav2_costmap_2d::StaticLayer>("static");
  addLayer<nav2_costmap_2d::InflationLayer>("inflation");
  // TODO(bpwilcox): replace manual footprint to layered_costmap with parameter or from nav2_robot
  setFootprint(0, 0);
  layered_costmap_->updateMap(0, 0, 0);

  // Create a service that will use the callback function to handle requests.
  costmapServer_ = create_service<nav2_msgs::srv::GetCostmap>(name + "_GetCostmap",
      std::bind(&CostmapWorldModel::costmap_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void CostmapWorldModel::costmap_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request>/*request*/,
  const std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received costmap request");

  nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
  rclcpp::Clock clock;

  response->map.metadata.size_x = costmap->getSizeInCellsX();
  response->map.metadata.size_y = costmap->getSizeInCellsY();
  response->map.metadata.resolution = costmap->getResolution();
  response->map.metadata.layer = "Master";
  response->map.metadata.map_load_time = now();
  response->map.metadata.update_time = now();

  tf2::Quaternion quaternion;
  // TODO(bpwilcox): Grab correct orientation information
  quaternion.setRPY(0.0, 0.0, 0.0);  // set roll, pitch, yaw
  response->map.metadata.origin.position.x = costmap->getOriginX();
  response->map.metadata.origin.position.y = costmap->getOriginY();
  response->map.metadata.origin.position.z = 0.0;
  response->map.metadata.origin.orientation = tf2::toMsg(quaternion);

  response->map.header.stamp = now();
  response->map.header.frame_id = "map";

  unsigned char * data = costmap->getCharMap();
  auto data_length = response->map.metadata.size_x * response->map.metadata.size_y;
  response->map.data.resize(data_length);
  response->map.data.assign(data, data + data_length);
}

void CostmapWorldModel::setFootprint(double length, double width)
{
  std::vector<geometry_msgs::msg::Point> polygon;
  geometry_msgs::msg::Point p;
  p.x = width / 2;
  p.y = length / 2;
  polygon.push_back(p);
  p.x = width / 2;
  p.y = -length / 2;
  polygon.push_back(p);
  p.x = -width / 2;
  p.y = -length / 2;
  polygon.push_back(p);
  p.x = -width / 2;
  p.y = length / 2;
  polygon.push_back(p);
  layered_costmap_->setFootprint(polygon);
}

CostmapWorldModel::CostmapWorldModel()
: CostmapWorldModel("WorldModel")
{
}

}  // namespace nav2_costmap_world_model
