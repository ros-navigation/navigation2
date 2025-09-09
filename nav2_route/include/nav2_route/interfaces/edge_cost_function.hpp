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

#ifndef NAV2_ROUTE__INTERFACES__EDGE_COST_FUNCTION_HPP_
#define NAV2_ROUTE__INTERFACES__EDGE_COST_FUNCTION_HPP_

#include <memory>
#include <string>

#include "tf2_ros/buffer.h"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_route/types.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"

namespace nav2_route
{

/**
 * @class EdgeCostFunction
 * @brief A plugin interface to score edges during graph search to modify
 * the lowest cost path (e.g. by distance, maximum speed, regions prefer not to travel
 * blocked by occupancy, or using arbitrarily defined user metadata stored in the
 * edge and nodes of interest.)
 */
class EdgeCostFunction
{
public:
  using Ptr = std::shared_ptr<nav2_route::EdgeCostFunction>;

  /**
   * @brief Constructor
   */
  EdgeCostFunction() = default;

  /**
   * @brief Virtual destructor
   */
  virtual ~EdgeCostFunction() = default;

  /**
   * @brief Configure the scorer, but do not store the node
   * @param parent pointer to user's node
   */
  virtual void configure(
    const nav2_util::LifecycleNode::SharedPtr node,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
    const std::string & name) = 0;

  /**
   * @brief Main scoring plugin API
   * @param edge The edge pointer to score, which has access to the
   * start/end nodes and their associated metadata and actions
   */
  virtual bool score(
    const EdgePtr edge, const RouteRequest & route_request,
    const EdgeType & edge_type, float & cost) = 0;

  /**
   * @brief Get name of the plugin for parameter scope mapping
   * @return Name
   */
  virtual std::string getName() = 0;

  /**
   * @brief Prepare for a new cycle, by resetting state, grabbing data
   * to use for all immediate requests, or otherwise prepare for scoring
   */
  virtual void prepare() {}
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__INTERFACES__EDGE_COST_FUNCTION_HPP_
