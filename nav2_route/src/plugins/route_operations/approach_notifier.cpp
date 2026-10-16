// Copyright (c) 2026 Suad Shabaj
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

#include <memory>
#include <optional>
#include <string>

#include "nav2_route/plugins/route_operations/approach_notifier.hpp"

namespace nav2_route
{

void ApproachNotifier::configure(
  const nav2::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/* costmap_subscriber */,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring Approach notifier operation.");
  name_ = name;
  logger_ = node->get_logger();
  approach_tag_ = node->declare_or_get_parameter(
    getName() + ".approach_tag", std::string("notify_on_approach"));
  approach_radius_ = static_cast<float>(node->declare_or_get_parameter(
    getName() + ".approach_radius", 2.0));

  if (approach_radius_ < 0.0f) {
    RCLCPP_WARN(node->get_logger(),
        "%s: approach_radius must be non-negative! Notifier won't trigger",
        name_.c_str());
  }

  std::string topic = node->declare_or_get_parameter(
    getName() + ".approach_topic", std::string("approached_node"));

  approach_pub_ = node->create_publisher<nav2_msgs::msg::RouteNode>(topic);
  approach_pub_->on_activate();
}

std::optional<float> ApproachNotifier::approachRadius(const NodePtr node)
{
  if (!node->metadata.getValue<bool>(approach_tag_, false)) {
    return std::nullopt;
  }
  return approach_radius_;
}

OperationResult ApproachNotifier::perform(
  NodePtr node_achieved,
  EdgePtr /*edge_entered*/,
  EdgePtr /*edge_exited*/,
  const Route & /*route*/,
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
  const Metadata * /*mdata*/)
{
  RCLCPP_INFO(logger_, "%s: Approaching node %u.", name_.c_str(), node_achieved->nodeid);

  auto msg = std::make_unique<nav2_msgs::msg::RouteNode>();
  msg->nodeid = node_achieved->nodeid;
  msg->position.x = node_achieved->coords.x;
  msg->position.y = node_achieved->coords.y;
  approach_pub_->publish(std::move(msg));
  return OperationResult();
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::ApproachNotifier, nav2_route::RouteOperation)
