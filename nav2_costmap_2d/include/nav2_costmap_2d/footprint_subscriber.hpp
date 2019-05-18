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

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace nav2_costmap_2d
{

class FootprintSubscriber
{
public:
  FootprintSubscriber(
    rclcpp::Node::SharedPtr ros_node,
    std::string & topic_name);
  ~FootprintSubscriber() {}

  bool getFootprint(std::vector<geometry_msgs::msg::Point> & footprint);

protected:
  void footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::string topic_name_;
  bool footprint_received_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__FOOTPRINT_SUBSCRIBER_HPP_
