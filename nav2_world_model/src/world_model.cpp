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
#include "nav2_world_model/world_model.hpp"

using std::vector;
using std::string;

namespace nav2_world_model
{

WorldModel::WorldModel(rclcpp::executor::Executor & executor, const string & name)
: Node(name),
  tfBuffer_(get_clock()),
  tfListener_(tfBuffer_)
{
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap", tfBuffer_);
  costmap_ = costmap_ros_->getCostmap();
  executor.add_node(costmap_ros_);

  // Create a service that will use the callback function to handle requests.
  costmapServer_ = create_service<nav2_msgs::srv::GetCostmap>("GetCostmap",
      std::bind(&WorldModel::costmap_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void WorldModel::costmap_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request>/*request*/,
  const std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received costmap request");

  response->map.metadata.size_x = costmap_->getSizeInCellsX();
  response->map.metadata.size_y = costmap_->getSizeInCellsY();
  response->map.metadata.resolution = costmap_->getResolution();
  response->map.metadata.layer = "Master";
  response->map.metadata.map_load_time = now();
  response->map.metadata.update_time = now();

  tf2::Quaternion quaternion;
  // TODO(bpwilcox): Grab correct orientation information
  quaternion.setRPY(0.0, 0.0, 0.0);  // set roll, pitch, yaw
  response->map.metadata.origin.position.x = costmap_->getOriginX();
  response->map.metadata.origin.position.y = costmap_->getOriginY();
  response->map.metadata.origin.position.z = 0.0;
  response->map.metadata.origin.orientation = tf2::toMsg(quaternion);

  response->map.header.stamp = now();
  response->map.header.frame_id = "map";

  unsigned char * data = costmap_->getCharMap();
  auto data_length = response->map.metadata.size_x * response->map.metadata.size_y;
  response->map.data.resize(data_length);
  response->map.data.assign(data, data + data_length);
}

WorldModel::WorldModel(rclcpp::executor::Executor & executor)
: WorldModel(executor, "world_model")
{
}

}  // namespace nav2_world_model
