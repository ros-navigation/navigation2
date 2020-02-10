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

#include "nav2_costmap_2d/costmap_subscriber.hpp"

namespace nav2_costmap_2d
{

CostmapSubscriber::CostmapSubscriber(
  nav2_util::LifecycleNode::SharedPtr node,
  const std::string & topic_name)
: CostmapSubscriber(node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_logging_interface(),
    topic_name)
{}

CostmapSubscriber::CostmapSubscriber(
  rclcpp::Node::SharedPtr node,
  const std::string & topic_name)
: CostmapSubscriber(node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_logging_interface(),
    topic_name)
{}

CostmapSubscriber::CostmapSubscriber(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  const std::string & topic_name)
: node_base_(node_base),
  node_topics_(node_topics),
  node_logging_(node_logging),
  topic_name_(topic_name)
{
  costmap_sub_ = rclcpp::create_subscription<nav2_msgs::msg::Costmap>(
    node_topics_, topic_name_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&CostmapSubscriber::costmapCallback, this, std::placeholders::_1));
}

std::shared_ptr<Costmap2D> CostmapSubscriber::getCostmap()
{
  if (!costmap_received_) {
    throw std::runtime_error("Costmap is not available");
  }
  toCostmap2D();
  return costmap_;
}

void CostmapSubscriber::toCostmap2D()
{
  if (costmap_ == nullptr) {
    costmap_ = std::make_shared<Costmap2D>(
      costmap_msg_->metadata.size_x, costmap_msg_->metadata.size_y,
      costmap_msg_->metadata.resolution, costmap_msg_->metadata.origin.position.x,
      costmap_msg_->metadata.origin.position.y);
  } else if (costmap_->getSizeInCellsX() != costmap_msg_->metadata.size_x ||  // NOLINT
    costmap_->getSizeInCellsY() != costmap_msg_->metadata.size_y ||
    costmap_->getResolution() != costmap_msg_->metadata.resolution ||
    costmap_->getOriginX() != costmap_msg_->metadata.origin.position.x ||
    costmap_->getOriginY() != costmap_msg_->metadata.origin.position.y)
  {
    // Update the size of the costmap
    costmap_->resizeMap(
      costmap_msg_->metadata.size_x, costmap_msg_->metadata.size_y,
      costmap_msg_->metadata.resolution,
      costmap_msg_->metadata.origin.position.x,
      costmap_msg_->metadata.origin.position.y);
  }

  unsigned char * master_array = costmap_->getCharMap();
  unsigned int index = 0;
  for (unsigned int i = 0; i < costmap_msg_->metadata.size_x; ++i) {
    for (unsigned int j = 0; j < costmap_msg_->metadata.size_y; ++j) {
      master_array[index] = costmap_msg_->data[index];
      ++index;
    }
  }
}

void CostmapSubscriber::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  costmap_msg_ = msg;
  if (!costmap_received_) {
    costmap_received_ = true;
  }
}

}  // namespace nav2_costmap_2d
