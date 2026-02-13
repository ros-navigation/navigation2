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
#include <memory>
#include <mutex>

#include "nav2_costmap_2d/costmap_subscriber.hpp"

namespace nav2_costmap_2d
{

CostmapSubscriber::CostmapSubscriber(
  const nav2::LifecycleNode::WeakPtr & parent,
  const std::string & topic_name)
: topic_name_(topic_name)
{
  auto node = parent.lock();
  logger_ = node->get_logger();

  costmap_sub_ = node->create_subscription<nav2_costmap_2d::Costmap2DStamped>(
    topic_name_,
    nav2::qos::LatchedSubscriptionQoS(3),
    std::bind(&CostmapSubscriber::costmapCallback, this, std::placeholders::_1));

  costmap_update_sub_ = node->create_subscription<nav2_msgs::msg::CostmapUpdate>(
    topic_name_ + "_updates",
    nav2::qos::LatchedSubscriptionQoS(),
    std::bind(&CostmapSubscriber::costmapUpdateCallback, this, std::placeholders::_1));
}

std::shared_ptr<Costmap2D> CostmapSubscriber::getCostmap()
{
  if (!isCostmapReceived()) {
    throw std::runtime_error("Costmap is not available");
  }
  return costmap_;
}

void CostmapSubscriber::costmapCallback(
  const std::shared_ptr<nav2_costmap_2d::Costmap2DStamped> msg)
{
  std::lock_guard<std::mutex> lock(costmap_msg_mutex_);
  if (!msg || !msg->costmap) {
    return;
  }
  costmap_ = msg->costmap;
  frame_id_ = msg->header.frame_id;
}

void CostmapSubscriber::costmapUpdateCallback(
  const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr & update_msg)
{
  if (isCostmapReceived()) {
    std::lock_guard<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    auto map_cell_size_x = costmap_->getSizeInCellsX();
    auto map_cell_size_y = costmap_->getSizeInCellsY();

    if (map_cell_size_x < update_msg->x + update_msg->size_x ||
      map_cell_size_y < update_msg->y + update_msg->size_y)
    {
      RCLCPP_WARN(
        logger_, "Update area outside of original map area. Costmap bounds: %d X %d, "
        "Update origin: %d, %d  bounds: %d X %d", map_cell_size_x, map_cell_size_y,
        update_msg->x, update_msg->y, update_msg->size_x, update_msg->size_y);
      return;
    }
    unsigned char * master_array = costmap_->getCharMap();
    for (size_t y = 0; y < update_msg->size_y; ++y) {
      auto starting_index_of_row_update_in_costmap = (y + update_msg->y) * map_cell_size_x +
        update_msg->x;

      std::copy_n(
        update_msg->data.begin() + (y * update_msg->size_x),
        update_msg->size_x, &master_array[starting_index_of_row_update_in_costmap]);
    }
  } else {
    RCLCPP_WARN(logger_, "No costmap received.");
  }
}

}  // namespace nav2_costmap_2d
