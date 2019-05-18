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
  rclcpp::Node::SharedPtr ros_node,
  std::string & topic_name)
: node_(ros_node), topic_name_(topic_name), footprint_received_(false)
{
  footprint_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(topic_name,
      std::bind(&FootprintSubscriber::footprint_callback, this, std::placeholders::_1));
}

bool
FootprintSubscriber::getFootprint(std::vector<geometry_msgs::msg::Point> & footprint)
{
  if (!footprint_received_) {
    return false;
  }
  footprint = footprint_;
  return true;
}

void
FootprintSubscriber::footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  if (!footprint_received_) {
    footprint_received_ = true;
  }

  footprint_ = toPointVector(
    std::make_shared<geometry_msgs::msg::Polygon>(msg->polygon));
}

}  // namespace nav2_costmap_2d
