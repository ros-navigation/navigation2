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

#ifndef NAV2_COSTMAP_2D__COSTMAP_SUBSCRIBER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_SUBSCRIBER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_costmap_2d
{

class CostmapSubscriber
{
public:
  CostmapSubscriber(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & topic_name,
    bool use_raw = false);

  CostmapSubscriber(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    bool use_raw = false);

  CostmapSubscriber(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    const std::string & topic_name,
    bool use_raw = false);

  ~CostmapSubscriber() {}

  std::shared_ptr<Costmap2D> getCostmap();

protected:
  // Interfaces used for logging and creating publishers and subscribers
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;

  void toCostmap2D();
  void costmapRawCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void costmapUpdateCallback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg);
  unsigned char interpretValue(unsigned char value);

  std::shared_ptr<Costmap2D> costmap_;
  nav2_msgs::msg::Costmap::SharedPtr costmap_raw_msg_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_msg_;
  map_msgs::msg::OccupancyGridUpdate::SharedPtr costmap_update_msg_;
  std::string topic_name_;
  bool costmap_received_{false};
  bool use_raw_;
  bool has_update_{false};
  bool track_unknown_space_{false};
  unsigned char lethal_threshold_{100};
  unsigned char unknown_cost_value_{static_cast<unsigned char>(0xff)};
  bool trinary_costmap_{true};
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_raw_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_update_sub_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_SUBSCRIBER_HPP_
