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

#include <string>

#include "nav2_costmap_2d/costmap_subscriber.hpp"

namespace nav2_costmap_2d
{

CostmapSubscriber::CostmapSubscriber(
  rclcpp::Node::SharedPtr ros_node,
  std::string & topic_name)
: node_(ros_node),
  topic_name_(topic_name),
  costmap_received_(false),
  costmap_(nullptr)
{
  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(topic_name,
      std::bind(&CostmapSubscriber::costmap_callback, this, std::placeholders::_1));
}

Costmap2D * CostmapSubscriber::getCostmap()
{
  if (costmap_ == nullptr) {
    throw std::runtime_error("Costmap is not available");
  }
  return costmap_;
}

void CostmapSubscriber::toCostmap2D()
{
  if (!costmap_received_) {
    costmap_ = new Costmap2D(
    msg_->metadata.size_x, msg_->metadata.size_y,
    msg_->metadata.resolution, msg_->metadata.origin.position.x,
    msg_->metadata.origin.position.y);
  } else if (costmap_->getSizeInCellsX() != msg_->metadata.size_x ||
    costmap_->getSizeInCellsY() != msg_->metadata.size_y ||
    costmap_->getResolution() != msg_->metadata.resolution ||
    costmap_->getOriginX() != msg_->metadata.origin.position.x ||
    costmap_->getOriginY() != msg_->metadata.origin.position.y)
  {
    // Update the size of the costmap 
    costmap_->resizeMap(msg_->metadata.size_x, msg_->metadata.size_y,
      msg_->metadata.resolution,
      msg_->metadata.origin.position.x,
      msg_->metadata.origin.position.y);
  }

  unsigned char * master_array = costmap_->getCharMap();
  unsigned int index = 0;
  for (unsigned int i = 0; i <  msg_->metadata.size_x; ++i) {
    for (unsigned int j = 0; j < msg_->metadata.size_y; ++j) {
      master_array[index] = msg_->data[index];
      ++index;
    }
  }
}

void CostmapSubscriber::costmap_callback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  msg_ = msg;
  toCostmap2D();
  if (!costmap_received_) {
    costmap_received_ = true;
  }
}

}  // namespace nav2_costmap_2d

