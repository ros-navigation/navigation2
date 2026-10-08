// Copyright (c) 2026 Yong Ling
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

#include "nav2_route/precomputed_route.hpp"

#include "nav2_core/route_exceptions.hpp"

namespace nav2_route
{

Route resolvePrecomputedRoute(
  Graph & graph, uint16_t start_node_id, const std::vector<uint16_t> & edge_ids)
{
  if (graph.empty()) {
    throw nav2_core::NoValidGraph("No route graph loaded");
  }

  NodePtr start = nullptr;
  for (auto & node : graph) {
    if (node.nodeid == start_node_id) {
      if (start) {
        throw nav2_core::NoValidRouteCouldBeFound("Ambiguous starting node ID");
      }
      start = &node;
    }
  }
  if (!start) {
    throw nav2_core::NoValidRouteCouldBeFound("Unknown starting node ID");
  }

  Route route;
  route.start_node = start;
  NodePtr current = start;
  for (const auto id : edge_ids) {
    EdgePtr selected = nullptr;
    for (auto & edge : current->neighbors) {
      if (edge.edgeid == id) {
        if (selected) {
          throw nav2_core::NoValidRouteCouldBeFound("Ambiguous outgoing edge ID");
        }
        selected = &edge;
      }
    }
    if (!selected || selected->start != current || !selected->end) {
      throw nav2_core::NoValidRouteCouldBeFound("Unknown or disconnected directed edge ID");
    }
    route.edges.push_back(selected);
    // This is the graph's stored cost, without invoking local planning or scorers.
    route.route_cost += selected->edge_cost.cost;
    current = selected->end;
  }
  return route;
}

}  // namespace nav2_route
