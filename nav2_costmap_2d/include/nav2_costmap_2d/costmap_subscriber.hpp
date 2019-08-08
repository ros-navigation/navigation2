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
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_costmap_2d
{

class CostmapSubscriber
{
public:
  CostmapSubscriber(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & topic_name);

  CostmapSubscriber(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name);

  CostmapSubscriber(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & topic_name);

  ~CostmapSubscriber() {}

  std::shared_ptr<Costmap2D> getCostmap();

protected:
  // Interfaces used for logging and creating publishers and subscribers
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

  void toCostmap2D();
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);

  std::shared_ptr<Costmap2D> costmap_;
  nav2_msgs::msg::Costmap::SharedPtr costmap_msg_;
  std::string topic_name_;
  bool costmap_received_{false};
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_SUBSCRIBER_HPP_
