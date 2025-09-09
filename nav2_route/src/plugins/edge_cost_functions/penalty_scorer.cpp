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

#include "nav2_route/plugins/edge_cost_functions/penalty_scorer.hpp"

namespace nav2_route
{

void PenaltyScorer::configure(
  const nav2_util::LifecycleNode::SharedPtr node,
  const std::shared_ptr<tf2_ros::Buffer>/* tf_buffer */,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/* costmap_subscriber */,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring penalty scorer.");
  name_ = name;

  // Find the tag at high the speed limit information is stored
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".penalty_tag", rclcpp::ParameterValue("penalty"));
  penalty_tag_ = node->get_parameter(getName() + ".penalty_tag").as_string();

  // Find the proportional weight to apply, if multiple cost functions
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".weight", rclcpp::ParameterValue(1.0));
  weight_ = static_cast<float>(node->get_parameter(getName() + ".weight").as_double());
}

bool PenaltyScorer::score(
  const EdgePtr edge,
  const RouteRequest & /* route_request */,
  const EdgeType & /* edge_type */, float & cost)
{
  // Get the speed limit, if set for an edge
  float penalty_val = 0.0f;
  penalty_val = edge->metadata.getValue<float>(penalty_tag_, penalty_val);
  cost = weight_ * penalty_val;
  return true;
}

std::string PenaltyScorer::getName()
{
  return name_;
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::PenaltyScorer, nav2_route::EdgeCostFunction)
