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

std::shared_ptr<Costmap2D> CostmapSubscriber::getCostmap()
{
  // Snapshot costmap_ under the mutex in a single critical section. The
  // TypeAdapter callback reassigns the shared_ptr on every message, so we
  // need to copy it under the lock to avoid a concurrent read/write race.
  // processCurrentCostmapMsg() mutates the underlying Costmap2D in place
  // (resizeMap + std::copy), so the snapshot reflects the updated data.
  std::shared_ptr<Costmap2D> snapshot;
  bool has_pending_msg;
  {
    std::lock_guard<std::mutex> lock(costmap_msg_mutex_);
    if (!costmap_) {
      throw std::runtime_error("Costmap is not available");
    }
    snapshot = costmap_;
    has_pending_msg = (costmap_msg_ != nullptr);
  }
  if (has_pending_msg) {
    processCurrentCostmapMsg();
  }
  return snapshot;
}

void CostmapSubscriber::costmapCallback(
  const std::shared_ptr<const nav2_costmap_2d::Costmap2DStamped> & msg)
{
  std::lock_guard<std::mutex> lock(costmap_msg_mutex_);
  if (!msg || !msg->costmap) {
    return;
  }
  costmap_ = msg->costmap;
  // Drop any pending legacy message — the TypeAdapter payload is the freshest.
  costmap_msg_.reset();
  frame_id_ = msg->header.frame_id;
}

void CostmapSubscriber::costmapCallback(const nav2_msgs::msg::Costmap::ConstSharedPtr & msg)
{
  {
    std::lock_guard<std::mutex> lock(costmap_msg_mutex_);
    costmap_msg_ = msg;
    frame_id_ = msg->header.frame_id;
  }
  // First message: allocate the backing Costmap2D and process eagerly so that
  // isCostmapReceived() reports true ASAP. Subsequent messages are only stored
  // and are processed lazily in getCostmap(); intermediate messages that are
  // never requested by a consumer cost nothing.
  if (!isCostmapReceived()) {
    {
      std::lock_guard<std::mutex> lock(costmap_msg_mutex_);
      costmap_ = std::make_shared<Costmap2D>(
        msg->metadata.size_x, msg->metadata.size_y,
        msg->metadata.resolution, msg->metadata.origin.position.x,
        msg->metadata.origin.position.y);
    }
    processCurrentCostmapMsg();
  }
}

void CostmapSubscriber::processCurrentCostmapMsg()
{
  std::scoped_lock lock(*(costmap_->getMutex()), costmap_msg_mutex_);
  if (!costmap_msg_) {
    return;
  }
  if (costmap_->getSizeInCellsX() != costmap_msg_->metadata.size_x ||
    costmap_->getSizeInCellsY() != costmap_msg_->metadata.size_y ||
    costmap_->getResolution() != costmap_msg_->metadata.resolution ||
    costmap_->getOriginX() != costmap_msg_->metadata.origin.position.x ||
    costmap_->getOriginY() != costmap_msg_->metadata.origin.position.y)
  {
    costmap_->resizeMap(
      costmap_msg_->metadata.size_x, costmap_msg_->metadata.size_y,
      costmap_msg_->metadata.resolution,
      costmap_msg_->metadata.origin.position.x,
      costmap_msg_->metadata.origin.position.y);
  }
  std::copy(costmap_msg_->data.begin(), costmap_msg_->data.end(), costmap_->getCharMap());
  costmap_msg_.reset();
}

void CostmapSubscriber::costmapUpdateCallback(
  const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr & update_msg)
{
  if (isCostmapReceived()) {
    // If a legacy full Costmap message is still pending, process it first so
    // the update is applied on top of the latest full snapshot. Otherwise a
    // later getCostmap() would overwrite the just-applied update with the
    // pending full message.
    bool has_pending_msg = false;
    {
      std::lock_guard<std::mutex> lock(costmap_msg_mutex_);
      has_pending_msg = (costmap_msg_ != nullptr);
    }
    if (has_pending_msg) {
      processCurrentCostmapMsg();
    }

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
    // copy update msg row-wise
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
