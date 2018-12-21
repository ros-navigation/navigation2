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

#include "nav2_world_model/costmap_representation.hpp"

namespace nav2_world_model
{

CostmapRepresentation::CostmapRepresentation(
  const std::string & name,
  rclcpp::executor::Executor & executor,
  rclcpp::Clock::SharedPtr & clock)
: name_(name),
  clock_(clock),
  tfBuffer_(clock_),
  tfListener_(tfBuffer_)
{
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(name_, tfBuffer_);
  costmap_ = costmap_ros_->getCostmap();
  executor.add_node(costmap_ros_);
}

GetCostmap::Response
CostmapRepresentation::getCostmap(const GetCostmap::Request & /*request*/)
{
  GetCostmap::Response response;
  response.map.metadata.size_x = costmap_->getSizeInCellsX();
  response.map.metadata.size_y = costmap_->getSizeInCellsY();
  response.map.metadata.resolution = costmap_->getResolution();
  response.map.metadata.layer = "Master";
  response.map.metadata.map_load_time = costmap_ros_->now();
  response.map.metadata.update_time = costmap_ros_->now();

  tf2::Quaternion quaternion;
  // TODO(bpwilcox): Grab correct orientation information
  quaternion.setRPY(0.0, 0.0, 0.0);  // set roll, pitch, yaw
  response.map.metadata.origin.position.x = costmap_->getOriginX();
  response.map.metadata.origin.position.y = costmap_->getOriginY();
  response.map.metadata.origin.position.z = 0.0;
  response.map.metadata.origin.orientation = tf2::toMsg(quaternion);

  response.map.header.stamp = costmap_ros_->now();
  response.map.header.frame_id = "map";

  unsigned char * data = costmap_->getCharMap();
  auto data_length = response.map.metadata.size_x * response.map.metadata.size_y;
  response.map.data.resize(data_length);
  response.map.data.assign(data, data + data_length);

  return response;
}

ProcessRegion::Response
CostmapRepresentation::confirmFreeSpace(const ProcessRegion::Request & /*request*/)
{
  // TODO(orduno)
  ProcessRegion::Response response;
  response.was_successful = false;
  return response;
}

ProcessRegion::Response
CostmapRepresentation::clearArea(const ProcessRegion::Request & /*request*/)
{
  // TODO(orduno)
  ProcessRegion::Response response;
  response.was_successful = false;
  return response;
}

}  // namespace nav2_world_model
