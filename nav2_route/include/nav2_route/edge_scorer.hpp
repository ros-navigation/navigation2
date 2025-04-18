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

#ifndef NAV2_ROUTE__EDGE_SCORER_HPP_
#define NAV2_ROUTE__EDGE_SCORER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "tf2_ros/buffer.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"
#include "nav2_route/interfaces/edge_cost_function.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace nav2_route
{

/**
 * @class nav2_route::EdgeScorer
 * @brief An class to encapsulate edge scoring logic for plugins and different user
 * specified algorithms to influence graph search. It has access to the edge, which
 * in turn has access to the parent and child node of the connection. It also contains
 * action and arbitrary user-defined metadata to enable edge scoring logic based on
 * arbitrary properties of the graph you select (e.g. some regions have a multiplier,
 * some actions are discouraged with higher costs like having to go through a door,
 * edges with reduced speed limits are proportionally less preferred for optimality
 * relative to the distance the edge represents to optimize time to goal)
 */
class EdgeScorer
{
public:
  /**
   * @brief Constructor
   */
  explicit EdgeScorer(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber);

  /**
   * @brief Destructor
   */
  ~EdgeScorer() = default;

  /**
   * @brief Score the edge with the set of plugins
   * @param edge Ptr to edge for scoring
   * @param goal_pose Pose Stamped of desired goal
   * @param score of edge
   * @param final_edge whether this edge brings us to the goal or not
   * @return If edge is valid
   */
  bool score(
    const EdgePtr edge, const RouteRequest & route_request,
    const EdgeType & edge_type,
    float & score);

  /**
   * @brief Provide the number of plugisn in the scorer loaded
   * @return Number of scoring plugins
   */
  int numPlugins() const;

protected:
  pluginlib::ClassLoader<EdgeCostFunction> plugin_loader_;
  std::vector<EdgeCostFunction::Ptr> plugins_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__EDGE_SCORER_HPP_
