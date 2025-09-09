// Copyright (c) 2025, Open Navigation LLC
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
#include <string>

#include "nav2_route/plugins/route_operations/adjust_speed_limit.hpp"

namespace nav2_route
{

void AdjustSpeedLimit::configure(
  const nav2_util::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/* costmap_subscriber */,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring Adjust speed limit operation.");
  name_ = name;
  logger_ = node->get_logger();
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".speed_tag", rclcpp::ParameterValue("speed_limit"));
  speed_tag_ = node->get_parameter(getName() + ".speed_tag").as_string();


  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".speed_limit_topic", rclcpp::ParameterValue("speed_limit"));
  std::string topic = node->get_parameter(getName() + ".speed_tag").as_string();

  speed_limit_pub_ = node->create_publisher<nav2_msgs::msg::SpeedLimit>(topic, 10);
  speed_limit_pub_->on_activate();
}

OperationResult AdjustSpeedLimit::perform(
  NodePtr /*node_achieved*/,
  EdgePtr edge_entered,
  EdgePtr /*edge_exited*/,
  const Route & /*route*/,
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
  const Metadata * /*mdata*/)
{
  OperationResult result;
  if (!edge_entered) {
    return result;
  }

  float speed_limit = 100.0;
  speed_limit = edge_entered->metadata.getValue<float>(speed_tag_, speed_limit);
  RCLCPP_DEBUG(logger_, "Setting speed limit to %.2f%% of maximum.", speed_limit);

  auto msg = std::make_unique<nav2_msgs::msg::SpeedLimit>();
  msg->percentage = true;
  msg->speed_limit = speed_limit;
  speed_limit_pub_->publish(std::move(msg));
  return result;
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::AdjustSpeedLimit, nav2_route::RouteOperation)
