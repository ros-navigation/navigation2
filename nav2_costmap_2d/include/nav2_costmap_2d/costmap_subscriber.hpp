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
#include "nav2_msgs/msg/costmap_update.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_costmap_2d
{
/**
 * @class CostmapSubscriber
 * @brief Subscribes to the costmap via a ros topic
 */
class CostmapSubscriber
{
public:
  /**
   * @brief A constructor
   */
  CostmapSubscriber(
    const nav2_util::LifecycleNode::WeakPtr & parent,
    const std::string & topic_name);

  /**
   * @brief A constructor
   */
  CostmapSubscriber(
    const rclcpp::Node::WeakPtr & parent,
    const std::string & topic_name);

  /**
   * @brief A destructor
   */
  ~CostmapSubscriber() {}

  /**
   * @brief Get current costmap
   */
  std::shared_ptr<Costmap2D> getCostmap();
  /**
   * @brief Callback for the costmap topic
   */
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  /**
   * @brief Callback for the costmap's update topic
   */
  void costmapUpdateCallback(const nav2_msgs::msg::CostmapUpdate::SharedPtr update_msg);

protected:
  bool isCostmapReceived() {return costmap_ != nullptr;}
  void processCurrentCostmapMsg();

  bool haveCostmapParametersChanged();
  bool hasCostmapSizeChanged();
  bool hasCostmapResolutionChanged();
  bool hasCostmapOriginPositionChanged();

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav2_msgs::msg::CostmapUpdate>::SharedPtr costmap_update_sub_;

  std::shared_ptr<Costmap2D> costmap_;
  nav2_msgs::msg::Costmap::SharedPtr costmap_msg_;

  std::string topic_name_;
  std::mutex costmap_msg_mutex_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_costmap_2d")};
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_SUBSCRIBER_HPP_
