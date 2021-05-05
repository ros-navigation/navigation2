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

#ifndef NAV2_COSTMAP_2D__FOOTPRINT_SUBSCRIBER_HPP_
#define NAV2_COSTMAP_2D__FOOTPRINT_SUBSCRIBER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_costmap_2d
{
/**
 * @class FootprintSubscriber
 * @brief Subscriber to the footprint topic to get current robot footprint
 * (if changing) for use in collision avoidance
 */
class FootprintSubscriber
{
public:
  /**
   * @brief A constructor
   */
  FootprintSubscriber(
    const nav2_util::LifecycleNode::WeakPtr & parent,
    const std::string & topic_name,
    const double & footprint_timeout);

  /**
   * @brief A constructor
   */
  FootprintSubscriber(
    const rclcpp::Node::WeakPtr & parent,
    const std::string & topic_name,
    const double & footprint_timeout);

  /**
   * @brief A destructor
   */
  ~FootprintSubscriber() {}

  /**
   * @brief Returns an oriented robot footprint at current time.
   */
  bool getFootprint(
    std::vector<geometry_msgs::msg::Point> & footprint,
    rclcpp::Duration & valid_footprint_timeout);
  /**
   * @brief Returns an oriented robot footprint without timeout
   */
  bool getFootprint(std::vector<geometry_msgs::msg::Point> & footprint);

  /**
   * @brief Returns an oriented robot footprint at stamped time.
   */
  bool getFootprint(
    std::vector<geometry_msgs::msg::Point> & footprint,
    rclcpp::Time & stamp, rclcpp::Duration valid_footprint_timeout);

protected:
  rclcpp::Clock::SharedPtr clock_;

  /**
   * @brief Callback to process new footprint updates.
   */
  void footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

  std::string topic_name_;
  bool footprint_received_{false};
  rclcpp::Duration footprint_timeout_;
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__FOOTPRINT_SUBSCRIBER_HPP_
