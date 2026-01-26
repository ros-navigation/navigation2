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
   * @brief A Get the costmap from topic
   */
  std::shared_ptr<Costmap2D> getCostmap();

  /**
   * @brief Convert an occ grid message into a costmap object
   */
  void toCostmap2D();
  /**
   * @brief Callback for the costmap topic
   */
  void costmapCallback(const std::shared_ptr<nav2_costmap_2d::Costmap2DStamped> msg);

protected:
  std::shared_ptr<Costmap2D> costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DStamped> costmap_msg_;
  std::string topic_name_;
  bool costmap_received_{false};
  rclcpp::Subscription<nav2_costmap_2d::Costmap2DStamped>::SharedPtr costmap_sub_;
};

}

#endif  // NAV2_COSTMAP_2D__COSTMAP_SUBSCRIBER_HPP_
