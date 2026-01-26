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
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_type_adapter.hpp"

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
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & topic_name);

  template<typename NodeT>
  CostmapSubscriber(
    const NodeT & parent,
    const std::string & topic_name,
    const rclcpp::CallbackGroup::SharedPtr & callback_group = nullptr)
  : topic_name_(topic_name)
  {
    logger_ = parent->get_logger();

    // Could be using a user rclcpp::Node, so need to use the Nav2 factory to create the
    // subscription to convert nav2::LifecycleNode, rclcpp::Node or rclcpp_lifecycle::LifecycleNode
    costmap_sub_ = nav2::interfaces::create_subscription<nav2_msgs::msg::Costmap>(
      parent, topic_name_,
      std::bind(&CostmapSubscriber::costmapCallback, this, std::placeholders::_1),
      nav2::qos::LatchedSubscriptionQoS(3), callback_group);

    costmap_update_sub_ = nav2::interfaces::create_subscription<nav2_msgs::msg::CostmapUpdate>(
      parent, topic_name_ + "_updates",
      std::bind(&CostmapSubscriber::costmapUpdateCallback, this, std::placeholders::_1),
      nav2::qos::LatchedSubscriptionQoS(), callback_group);
  }

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
  void costmapCallback(const std::shared_ptr<nav2_costmap_2d::Costmap2DStamped> msg);

protected:
  bool isCostmapReceived() {return costmap_ != nullptr;}
  void processCurrentCostmapMsg();

  bool haveCostmapParametersChanged();
  bool hasCostmapSizeChanged();
  bool hasCostmapResolutionChanged();
  bool hasCostmapOriginPositionChanged();

  nav2::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2::Subscription<nav2_msgs::msg::CostmapUpdate>::SharedPtr costmap_update_sub_;

  std::shared_ptr<Costmap2D> costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DStamped> costmap_msg_;
  std::string topic_name_;
  bool costmap_received_{false};
  rclcpp::Subscription<nav2_costmap_2d::Costmap2DStamped>::SharedPtr costmap_sub_;
};

}

#endif  // NAV2_COSTMAP_2D__COSTMAP_SUBSCRIBER_HPP_
