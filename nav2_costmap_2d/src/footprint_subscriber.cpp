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

#include "nav2_costmap_2d/footprint_subscriber.hpp"

namespace nav2_costmap_2d
{

FootprintSubscriber::FootprintSubscriber(
  nav2_util::LifecycleNode::SharedPtr node,
  std::string & topic_name, rclcpp::QoS qos)
: SimpleSubscriber<geometry_msgs::msg::PolygonStamped>(node, topic_name, qos)
{}

FootprintSubscriber::FootprintSubscriber(
  rclcpp::Node::SharedPtr node,
  std::string & topic_name, rclcpp::QoS qos)
: SimpleSubscriber<geometry_msgs::msg::PolygonStamped>(node, topic_name, qos)
{}

bool
FootprintSubscriber::getFootprint(std::vector<geometry_msgs::msg::Point> & footprint)
{
  if (!received_) {
    return false;
  }

  footprint = toPointVector(
    std::make_shared<geometry_msgs::msg::Polygon>(msg_->polygon));
  return true;
}

}  // namespace nav2_costmap_2d
