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
#include <vector>
#include <memory>

#include "nav2_costmap_2d/footprint_subscriber.hpp"

namespace nav2_costmap_2d
{

FootprintSubscriber::FootprintSubscriber(
  nav2_util::LifecycleNode::SharedPtr node,
  const std::string & topic_name,
  const double & footprint_timeout)
: FootprintSubscriber(node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_logging_interface(),
    node->get_node_clock_interface(),
    topic_name, footprint_timeout)
{}

FootprintSubscriber::FootprintSubscriber(
  rclcpp::Node::SharedPtr node,
  const std::string & topic_name,
  const double & footprint_timeout)
: FootprintSubscriber(node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_logging_interface(),
    node->get_node_clock_interface(),
    topic_name, footprint_timeout)
{}

FootprintSubscriber::FootprintSubscriber(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
  const std::string & topic_name,
  const double & footprint_timeout)
: node_base_(node_base),
  node_topics_(node_topics),
  node_logging_(node_logging),
  node_clock_(node_clock),
  topic_name_(topic_name),
  footprint_timeout_(rclcpp::Duration::from_seconds(footprint_timeout))
{
  footprint_sub_ = rclcpp::create_subscription<geometry_msgs::msg::PolygonStamped>(
    node_topics_,
    topic_name, rclcpp::SystemDefaultsQoS(),
    std::bind(&FootprintSubscriber::footprint_callback, this, std::placeholders::_1));
}

bool
FootprintSubscriber::getFootprint(
  std::vector<geometry_msgs::msg::Point> & footprint,
  rclcpp::Time & stamp,
  rclcpp::Duration valid_footprint_timeout)
{
  if (!footprint_received_) {
    return false;
  }

  footprint = toPointVector(
    std::make_shared<geometry_msgs::msg::Polygon>(footprint_->polygon));
  auto & footprint_stamp = footprint_->header.stamp;

  if (stamp - footprint_stamp > valid_footprint_timeout) {
    return false;
  }

  return true;
}

bool
FootprintSubscriber::getFootprint(
  std::vector<geometry_msgs::msg::Point> & footprint,
  rclcpp::Duration & valid_footprint_timeout)
{
  rclcpp::Time t = node_clock_->get_clock()->now();
  return getFootprint(footprint, t, valid_footprint_timeout);
}

bool
FootprintSubscriber::getFootprint(
  std::vector<geometry_msgs::msg::Point> & footprint)
{
  return getFootprint(footprint, footprint_timeout_);
}

void
FootprintSubscriber::footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  footprint_ = msg;
  if (!footprint_received_) {
    footprint_received_ = true;
  }
}

}  // namespace nav2_costmap_2d
