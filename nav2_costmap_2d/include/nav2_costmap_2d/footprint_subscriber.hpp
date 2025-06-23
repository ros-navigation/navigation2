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
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"

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
  template<typename NodeT>
  explicit FootprintSubscriber(
    const NodeT & parent,
    const std::string & topic_name,
    tf2_ros::Buffer & tf,
    std::string robot_base_frame = "base_link",
    double transform_tolerance = 0.1)
  : tf_(tf),
    robot_base_frame_(robot_base_frame),
    transform_tolerance_(transform_tolerance)
  {
    // Could be using a user rclcpp::Node, so need to use the Nav2 factory to create the
    // subscription to convert nav2::LifecycleNode, rclcpp::Node or rclcpp_lifecycle::LifecycleNode
    footprint_sub_ = nav2::interfaces::create_subscription<geometry_msgs::msg::PolygonStamped>(
      parent, topic_name,
      std::bind(&FootprintSubscriber::footprint_callback, this, std::placeholders::_1));
  }

  /**
   * @brief A destructor
   */
  ~FootprintSubscriber() {}

  /**
   * @brief Returns the latest robot footprint, in the form as received from topic (oriented).
   *
   * @param footprint Output param. Latest received footprint
   * @param footprint_header Output param. Header associated with the footprint
   * @return False if no footprint has been received
   */
  bool getFootprintRaw(
    std::vector<geometry_msgs::msg::Point> & footprint,
    std_msgs::msg::Header & footprint_header);

  /**
   * @brief Returns the latest robot footprint, transformed into robot base frame (unoriented).
   *
   * @param footprint Output param. Latest received footprint, unoriented
   * @param footprint_header Output param. Header associated with the footprint
   * @return False if no footprint has been received or if transformation failed
   */
  bool getFootprintInRobotFrame(
    std::vector<geometry_msgs::msg::Point> & footprint,
    std_msgs::msg::Header & footprint_header);

protected:
  /**
   * @brief Callback to process new footprint updates.
   */
  void footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

  tf2_ros::Buffer & tf_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  bool footprint_received_{false};
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint_;
  nav2::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__FOOTPRINT_SUBSCRIBER_HPP_
