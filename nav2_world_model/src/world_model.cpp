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
#include "nav2_world_model/costmap_representation.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

using std::vector;
using std::string;

namespace nav2_world_model
{

WorldModel::WorldModel(rclcpp::executor::Executor & executor)
: Node("world_model")
{
  auto clock = get_clock();
  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](auto) {});

  // Use Costmaps to represent the world, one with static objects from map
  world_representations_["global_costmap"] = std::make_unique<CostmapRepresentation>(
    "global_costmap", temp_node, executor, clock, "map");

  // And another centered at the robot with only objects detected by sensors
  world_representations_["robot_centric_costmap"] = std::make_unique<CostmapRepresentation>(
    "robot_centric_costmap", temp_node, executor, clock, "base_link");

  get_costmap_service_ = create_service<GetCostmap>("GetCostmap",
      std::bind(&WorldModel::getCostmapCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  confirm_free_space_service_ = create_service<ProcessRegion>("ConfirmFreeSpace",
      std::bind(&WorldModel::confirmFreeSpaceCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void WorldModel::getCostmapCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<GetCostmap::Request> request,
  std::shared_ptr<GetCostmap::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received costmap request");

  *response = world_representations_["global_costmap"]->getCostmap(*request);

  RCLCPP_INFO(get_logger(), "Sending costmap of size %d, %d",
    response->map.metadata.size_x, response->map.metadata.size_y);
}

void WorldModel::confirmFreeSpaceCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ProcessRegion::Request> request,
  std::shared_ptr<ProcessRegion::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received confirm free space request");

  std::string frame_id = request->frame_id;

  if ("base_link" == frame_id) {
    *response = world_representations_["robot_centric_costmap"]->confirmFreeSpace(*request);
  } else if ("map" == frame_id) {
    *response = world_representations_["global_costmap"]->confirmFreeSpace(*request);
  } else {
    RCLCPP_WARN(get_logger(), "Reference frame %s not supported", frame_id_.c_str());
  }
}

}  // namespace nav2_world_model
