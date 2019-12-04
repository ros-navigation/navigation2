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
#include "nav2_costmap_2d/cost_values.hpp"

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_costmap_2d
{

CostmapSubscriber::CostmapSubscriber(
  nav2_util::LifecycleNode::SharedPtr node,
  const std::string & topic_name,
  bool use_raw)
: CostmapSubscriber(node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_logging_interface(),
    node->get_node_parameters_interface(),
    topic_name,
    use_raw)
{}

CostmapSubscriber::CostmapSubscriber(
  rclcpp::Node::SharedPtr node,
  const std::string & topic_name,
  bool use_raw)
: CostmapSubscriber(node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_logging_interface(),
    node->get_node_parameters_interface(),
    topic_name,
    use_raw)
{}

CostmapSubscriber::CostmapSubscriber(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
  const std::string & topic_name,
  bool use_raw)
: node_base_(node_base),
  node_topics_(node_topics),
  node_logging_(node_logging),
  node_parameters_(node_parameters),
  topic_name_(topic_name),
  use_raw_(use_raw)
{
  if (use_raw_) {
    costmap_raw_sub_ = rclcpp::create_subscription<nav2_msgs::msg::Costmap>(node_topics_,
        topic_name_,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&CostmapSubscriber::costmapRawCallback, this, std::placeholders::_1));
  } else {
    costmap_sub_ = rclcpp::create_subscription<nav_msgs::msg::OccupancyGrid>(node_topics_,
        topic_name_,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&CostmapSubscriber::costmapCallback, this, std::placeholders::_1));
    costmap_update_sub_ = rclcpp::create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      node_topics_,
      topic_name_,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&CostmapSubscriber::costmapUpdateCallback, this, std::placeholders::_1));
    rclcpp::Parameter p;
    if (node_parameters_->get_parameter("track_unknown_space", p)) {
      track_unknown_space_ = p.get_value<bool>();
    }
    if (node_parameters_->get_parameter("lethal_cost_threshold", p)) {
      lethal_threshold_ = p.get_value<unsigned char>();
    }
    if (node_parameters_->get_parameter("unknown_cost_value", p)) {
      unknown_cost_value_ = p.get_value<unsigned char>();
    }
    if (node_parameters_->get_parameter("trinary_costmap", p)) {
      trinary_costmap_ = p.get_value<bool>();
    }
  }
}

std::shared_ptr<Costmap2D> CostmapSubscriber::getCostmap()
{
  if (!costmap_received_) {
    throw std::runtime_error("Costmap is not available");
  }
  return costmap_;
}

void CostmapSubscriber::toCostmap2D()
{
  if (use_raw_) {
    if (costmap_ == nullptr) {
      costmap_ = std::make_shared<Costmap2D>(
        costmap_raw_msg_->metadata.size_x, costmap_raw_msg_->metadata.size_y,
        costmap_raw_msg_->metadata.resolution, costmap_raw_msg_->metadata.origin.position.x,
        costmap_raw_msg_->metadata.origin.position.y);
    } else if (costmap_->getSizeInCellsX() != costmap_raw_msg_->metadata.size_x ||  // NOLINT
      costmap_->getSizeInCellsY() != costmap_raw_msg_->metadata.size_y ||
      costmap_->getResolution() != costmap_raw_msg_->metadata.resolution ||
      costmap_->getOriginX() != costmap_raw_msg_->metadata.origin.position.x ||
      costmap_->getOriginY() != costmap_raw_msg_->metadata.origin.position.y)
    {
      // Update the size of the costmap
      costmap_->resizeMap(costmap_raw_msg_->metadata.size_x, costmap_raw_msg_->metadata.size_y,
        costmap_raw_msg_->metadata.resolution,
        costmap_raw_msg_->metadata.origin.position.x,
        costmap_raw_msg_->metadata.origin.position.y);
    }

    unsigned char * master_array = costmap_->getCharMap();
    unsigned int index = 0;
    for (unsigned int i = 0; i < costmap_raw_msg_->metadata.size_x; ++i) {
      for (unsigned int j = 0; j < costmap_raw_msg_->metadata.size_y; ++j) {
        master_array[index] = costmap_raw_msg_->data[index];
        ++index;
      }
    }
  } else {
    if (has_update_) {
      unsigned char * master_array = costmap_->getCharMap();
      unsigned int di = 0;
      for (unsigned int y = 0; y < costmap_update_msg_->height; y++) {
        unsigned int index_base = (costmap_update_msg_->y + y) * costmap_msg_->info.width;
        for (unsigned int x = 0; x < costmap_update_msg_->width; x++) {
          unsigned int index = index_base + x + costmap_update_msg_->x;
          master_array[index] = interpretValue(costmap_update_msg_->data[di++]);
        }
      }
      has_update_ = false;
    } else {
      if (costmap_ == nullptr) {
        costmap_ = std::make_shared<Costmap2D>(
          costmap_msg_->info.width, costmap_msg_->info.height,
          costmap_msg_->info.resolution, costmap_msg_->info.origin.position.x,
          costmap_msg_->info.origin.position.y);
      } else if (costmap_->getSizeInCellsX() != costmap_msg_->info.width ||  // NOLINT
        costmap_->getSizeInCellsY() != costmap_msg_->info.height ||
        costmap_->getResolution() != costmap_msg_->info.resolution ||
        costmap_->getOriginX() != costmap_msg_->info.origin.position.x ||
        costmap_->getOriginY() != costmap_msg_->info.origin.position.y)
      {
        // Update the size of the costmap
        costmap_->resizeMap(costmap_msg_->info.width, costmap_msg_->info.height,
          costmap_msg_->info.resolution,
          costmap_msg_->info.origin.position.x,
          costmap_msg_->info.origin.position.y);
      }

      unsigned char * master_array = costmap_->getCharMap();
      unsigned int index = 0;
      for (unsigned int i = 0; i < costmap_msg_->info.width; ++i) {
        for (unsigned int j = 0; j < costmap_msg_->info.height; ++j) {
          unsigned char value = costmap_msg_->data[index];
          master_array[index] = interpretValue(value);
          ++index;
        }
      }
    }
  }
}

unsigned char
CostmapSubscriber::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_) {
    return NO_INFORMATION;
  } else if (!track_unknown_space_ && value == unknown_cost_value_) {
    return FREE_SPACE;
  } else if (value >= lethal_threshold_) {
    return LETHAL_OBSTACLE;
  } else if (trinary_costmap_) {
    return FREE_SPACE;
  }

  double scale = static_cast<double>(value) / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void CostmapSubscriber::costmapRawCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  costmap_raw_msg_ = msg;
  if (!costmap_received_) {
    costmap_received_ = true;
  }
  toCostmap2D();
}

void CostmapSubscriber::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  costmap_msg_ = msg;
  if (!costmap_received_) {
    costmap_received_ = true;
  }
  toCostmap2D();
}

void CostmapSubscriber::costmapUpdateCallback(
  const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
{
  costmap_update_msg_ = msg;
  if (!costmap_received_) {
    costmap_received_ = true;
  }
  has_update_ = true;
  toCostmap2D();
}

}  // namespace nav2_costmap_2d
