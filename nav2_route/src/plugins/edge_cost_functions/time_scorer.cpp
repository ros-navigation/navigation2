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

#include "nav2_route/plugins/edge_cost_functions/time_scorer.hpp"

namespace nav2_route
{

void TimeScorer::configure(
  const nav2_util::LifecycleNode::SharedPtr node,
  const std::shared_ptr<tf2_ros::Buffer>/* tf_buffer */,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/* costmap_subscriber */,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring time scorer.");
  name_ = name;

  // Find the tag at high the speed limit information is stored
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".speed_tag", rclcpp::ParameterValue("abs_speed_limit"));
  speed_tag_ = node->get_parameter(getName() + ".speed_tag").as_string();

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".time_tag", rclcpp::ParameterValue("abs_time_taken"));
  prev_time_tag_ = node->get_parameter(getName() + ".time_tag").as_string();

  // Find the proportional weight to apply, if multiple cost functions
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".weight", rclcpp::ParameterValue(1.0));
  weight_ = static_cast<float>(node->get_parameter(getName() + ".weight").as_double());

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".max_vel", rclcpp::ParameterValue(0.5));
  max_vel_ = static_cast<float>(node->get_parameter(getName() + ".max_vel").as_double());
}

bool TimeScorer::score(
  const EdgePtr edge,
  const RouteRequest & /* route_request */,
  const EdgeType & /* edge_type */, float & cost)
{
  // We have a previous actual time to utilize for a refined estimate
  float time = 0.0;
  time = edge->metadata.getValue<float>(prev_time_tag_, time);
  if (time > 0.0f) {
    cost = weight_ * time;
    return true;
  }

  // Else: Get the speed limit, if set for an edge, else use max velocity
  float velocity = 0.0f;
  velocity = edge->metadata.getValue<float>(speed_tag_, velocity);
  if (velocity <= 0.0f) {
    velocity = max_vel_;
  }

  cost = weight_ * hypotf(
    edge->end->coords.x - edge->start->coords.x,
    edge->end->coords.y - edge->start->coords.y) / velocity;
  return true;
}

std::string TimeScorer::getName()
{
  return name_;
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::TimeScorer, nav2_route::EdgeCostFunction)
