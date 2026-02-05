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

#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"

namespace nav2_costmap_2d
{

CostmapSubscriber::CostmapSubscriber(
  const nav2::LifecycleNode::WeakPtr & parent,
  const std::string & topic_name)
: topic_name_(topic_name)
{
  auto node = parent.lock();
  costmap_sub_ = node->create_subscription<nav2_costmap_2d::Costmap2DStamped>(
    topic_name_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&CostmapSubscriber::costmapCallback, this, std::placeholders::_1));
}

CostmapSubscriber::CostmapSubscriber(
  const rclcpp::Node::WeakPtr & parent,
  const std::string & topic_name)
: topic_name_(topic_name)
{
  auto node = parent.lock();
  costmap_sub_ = node->create_subscription<nav2_costmap_2d::Costmap2DStamped>(
    topic_name_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&CostmapSubscriber::costmapCallback, this, std::placeholders::_1));
}

std::shared_ptr<Costmap2D> CostmapSubscriber::getCostmap()
{
  if (!isCostmapReceived()) {
    throw std::runtime_error("Costmap is not available");
  }
  if (costmap_msg_) {
    processCurrentCostmapMsg();
  }
  return costmap_;
}

void CostmapSubscriber::costmapCallback(const nav2_msgs::msg::Costmap::ConstSharedPtr & msg)
{
  auto current = std::atomic_load(&costmap_msg_);
  if (!current || !current->costmap) {
    return;
  }
  costmap_ = current->costmap;
}

void CostmapSubscriber::costmapCallback(
  const std::shared_ptr<nav2_costmap_2d::Costmap2DStamped> msg)
{
  return hasCostmapSizeChanged() ||
         hasCostmapResolutionChanged() ||
         hasCostmapOriginPositionChanged();
}

bool CostmapSubscriber::hasCostmapSizeChanged()
{
  return costmap_->getSizeInCellsX() != costmap_msg_->metadata.size_x ||
         costmap_->getSizeInCellsY() != costmap_msg_->metadata.size_y;
}

bool CostmapSubscriber::hasCostmapResolutionChanged()
{
  return costmap_->getResolution() != costmap_msg_->metadata.resolution;
}

bool CostmapSubscriber::hasCostmapOriginPositionChanged()
{
  return costmap_->getOriginX() != costmap_msg_->metadata.origin.position.x ||
         costmap_->getOriginY() != costmap_msg_->metadata.origin.position.y;
}

}  // namespace nav2_costmap_2d
