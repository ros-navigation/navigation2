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

#ifndef NAV2_ROUTE__PRECOMPUTED_ROUTE_HPP_
#define NAV2_ROUTE__PRECOMPUTED_ROUTE_HPP_

#include <cstdint>
#include <vector>

#include "nav2_route/types.hpp"

namespace nav2_route
{

/**
 * @brief Resolve an exact directed edge sequence using canonical graph objects.
 * @throws nav2_core::NoValidGraph if the graph is empty.
 * @throws nav2_core::NoValidRouteCouldBeFound for unknown, ambiguous, or disconnected IDs.
 * The caller must keep the graph alive and prevent replacement while using the route.
 * Loops and repeated edges are preserved. An empty edge sequence is a single-node route.
 */
Route resolvePrecomputedRoute(
  Graph & graph, uint16_t start_node_id, const std::vector<uint16_t> & edge_ids);

}  // namespace nav2_route

#endif  // NAV2_ROUTE__PRECOMPUTED_ROUTE_HPP_
