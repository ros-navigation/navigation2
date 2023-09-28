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
  const nav2_util::LifecycleNode::WeakPtr & parent,
  const std::string & topic_name)
: topic_name_(topic_name)
{
  auto node = parent.lock();
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    topic_name_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&CostmapSubscriber::costmapCallback, this, std::placeholders::_1));
  costmap_update_sub_ = node->create_subscription<nav2_msgs::msg::CostmapUpdate>(
    topic_name_+ "_updates",
    rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable(),
    std::bind(&CostmapSubscriber::costmapUpdateCallback, this, std::placeholders::_1));
}


// todo: is it still necessary
CostmapSubscriber::CostmapSubscriber(
  const rclcpp::Node::WeakPtr & parent,
  const std::string & topic_name)
: topic_name_(topic_name)
{
  auto node = parent.lock();
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    topic_name_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&CostmapSubscriber::costmapCallback, this, std::placeholders::_1));
  costmap_update_sub_ = node->create_subscription<nav2_msgs::msg::CostmapUpdate>(
    topic_name_+ "_updates",
    rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable(),
    std::bind(&CostmapSubscriber::costmapUpdateCallback, this, std::placeholders::_1));
}

// todo: change to const?
std::shared_ptr<Costmap2D> CostmapSubscriber::getCostmap()
{
  if (!costmap_received_) {
    throw std::runtime_error("Costmap is not available");
  }
  if(costmap_msg_ != nullptr)
  {
    processCurrentCostmapMsg(); 
  }
  return costmap_;
}

void CostmapSubscriber::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(costmap_msg_mutex_); 
    costmap_msg_ = msg;
  }
  if (!costmap_received_) {
    costmap_ = std::make_shared<Costmap2D>(
      msg->metadata.size_x, msg->metadata.size_y,
      msg->metadata.resolution, msg->metadata.origin.position.x,
      msg->metadata.origin.position.y);

      processCurrentCostmapMsg();

      costmap_received_ = true;
  }
}

void CostmapSubscriber::costmapUpdateCallback(const nav2_msgs::msg::CostmapUpdate::SharedPtr update_msg)
{
  if(costmap_received_)
  {
    if(costmap_msg_ != nullptr)
    {
     processCurrentCostmapMsg(); 
    }

    std::lock_guard<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    if(costmap_->getSizeInCellsX() < update_msg->x + update_msg->size_x ||
      costmap_->getSizeInCellsY() < update_msg->y + update_msg->size_y)
      {
        // todo : log error + error when frames dont match
        return;
      }
    // for (size_t y = 0; y < update_msg->height; y++)
    // {
    //   std::copy_n(v_in.begin(), 100, v_out.begin());
    //   memcpy(&current_map_.data[(update->y + y) * current_map_.info.width + update->x],
    //         &update->data[y * update->width], update->width);
    // }
  }
  else
  {
    // todo: log debug
  }
}

void CostmapSubscriber::processCurrentCostmapMsg()
{ 
  std::scoped_lock lock(*(costmap_->getMutex()), costmap_msg_mutex_); 
  if(areCostmapParametersChanged())
  {
    costmap_->resizeMap(
      costmap_msg_->metadata.size_x, costmap_msg_->metadata.size_y,
      costmap_msg_->metadata.resolution,
      costmap_msg_->metadata.origin.position.x,
      costmap_msg_->metadata.origin.position.y);
  }
  
  auto master_array = costmap_->getCharMap();
  std::copy(costmap_msg_->data.begin(), costmap_msg_->data.end(), master_array);
  costmap_msg_ = nullptr;
}

bool CostmapSubscriber::areCostmapParametersChanged()
{
  return costmap_->getSizeInCellsX() != costmap_msg_->metadata.size_x ||  
    costmap_->getSizeInCellsY() != costmap_msg_->metadata.size_y ||
    costmap_->getResolution() != costmap_msg_->metadata.resolution ||
    costmap_->getOriginX() != costmap_msg_->metadata.origin.position.x ||
    costmap_->getOriginY() != costmap_msg_->metadata.origin.position.y;
}

}  // namespace nav2_costmap_2d
