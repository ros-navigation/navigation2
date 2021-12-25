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
  const nav2_util::LifecycleNode::WeakPtr & parent,
  const std::string & topic_name,
  const double & footprint_timeout)
: topic_name_(topic_name),
  footprint_timeout_(rclcpp::Duration::from_seconds(footprint_timeout))
{
  auto node = parent.lock();
  clock_ = node->get_clock();
  footprint_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    topic_name, rclcpp::SystemDefaultsQoS(),
    std::bind(&FootprintSubscriber::footprint_callback, this, std::placeholders::_1));
}

FootprintSubscriber::FootprintSubscriber(
  const rclcpp::Node::WeakPtr & parent,
  const std::string & topic_name,
  const double & footprint_timeout)
: topic_name_(topic_name),
  footprint_timeout_(rclcpp::Duration::from_seconds(footprint_timeout))
{
  auto node = parent.lock();
  clock_ = node->get_clock();
  footprint_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
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

  auto current_footprint = std::atomic_load(&footprint_);
  footprint = toPointVector(
    std::make_shared<geometry_msgs::msg::Polygon>(current_footprint->polygon));
  auto & footprint_stamp = current_footprint->header.stamp;

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
  rclcpp::Time t = clock_->now();
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
  std::atomic_store(&footprint_, msg);
  if (!footprint_received_) {
    footprint_received_ = true;
  }
}

}  // namespace nav2_costmap_2d
