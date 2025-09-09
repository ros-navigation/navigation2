// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_ROUTE__PLUGINS__EDGE_COST_FUNCTIONS__PENALTY_SCORER_HPP_
#define NAV2_ROUTE__PLUGINS__EDGE_COST_FUNCTIONS__PENALTY_SCORER_HPP_

#include <memory>
#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_route/interfaces/edge_cost_function.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_route
{

/**
 * @class PenaltyScorer
 * @brief Adjusts the score of an edge by an amount set as metadata in the graph
 */
class PenaltyScorer : public EdgeCostFunction
{
public:
  /**
   * @brief Constructor
   */
  PenaltyScorer() = default;

  /**
   * @brief destructor
   */
  virtual ~PenaltyScorer() = default;

  /**
   * @brief Configure
   */
  void configure(
    const nav2_util::LifecycleNode::SharedPtr node,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
    const std::string & name) override;

  /**
   * @brief Main scoring plugin API
   * @param edge The edge pointer to score, which has access to the
   * start/end nodes and their associated metadata and actions
   * @param cost of the edge scored
   * @return bool if this edge is open valid to traverse
   */
  bool score(
    const EdgePtr edge, const RouteRequest & route_request,
    const EdgeType & edge_type, float & cost) override;

  /**
   * @brief Get name of the plugin for parameter scope mapping
   * @return Name
   */
  std::string getName() override;

protected:
  std::string name_;
  std::string penalty_tag_;
  float weight_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__PLUGINS__EDGE_COST_FUNCTIONS__PENALTY_SCORER_HPP_
