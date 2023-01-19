// Copyright (c) 2023, Samsung Research America
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

#include "nav2_route/plugins/edge_cost_functions/closed_edge_scorer.hpp"

namespace nav2_route
{

void ClosedEdgeScorer::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name)
{
  name_ = name;
  service_ =
    node->create_service<nav2_msgs::srv::ModifyClosedEdges>(
    getName() + "/closed_edges", std::bind(
      &ClosedEdgeScorer::closedEdgesCb, this,
      std::placeholders::_1, std::placeholders::_2));
}

void ClosedEdgeScorer::closedEdgesCb(
  const std::shared_ptr<nav2_msgs::srv::ModifyClosedEdges::Request> request,
  std::shared_ptr<nav2_msgs::srv::ModifyClosedEdges::Response> response)
{
  // Add new closed edges
  for (unsigned int edge : request->closed_edges) {
    closed_edges_.insert(edge);
  }

  // Removed now opened edges, if stored
  for (unsigned int edge : request->opened_edges) {
    if (closed_edges_.find(edge) != closed_edges_.end()) {
      closed_edges_.erase(edge);
    }
  }

  response->success = true;
}

bool ClosedEdgeScorer::score(const EdgePtr edge, float & /*cost*/)
{
  // Find if this edge is in the closed set of edges
  if (closed_edges_.find(edge->edgeid) != closed_edges_.end()) {
    return false;
  }

  return true;
}

std::string ClosedEdgeScorer::getName()
{
  return name_;
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::ClosedEdgeScorer, nav2_route::EdgeCostFunction)
