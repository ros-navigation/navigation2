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

#ifndef NAV2_ROUTE__INTERFACES__ROUTE_OPERATION_HPP_
#define NAV2_ROUTE__INTERFACES__ROUTE_OPERATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_route/types.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"

namespace nav2_route
{

/**
 * @struct OperationResult
 * @brief a struct to hold return from an operation
 */
struct OperationResult
{
  bool reroute{false};
  std::vector<unsigned int> blocked_ids;
};

/**
 * @enum nav2_route::RouteOperationType
 * @brief The type of operation plugin
 */
enum class RouteOperationType
{
  ON_GRAPH = 0,
  ON_STATUS_CHANGE = 1,
  ON_QUERY = 2
};

/**
 * @class RouteOperation
 * @brief A plugin interface to perform an operation while tracking the route
 * such as triggered from the graph file when a particular node is achieved,
 * edge is entered or exited. The API also supports triggering arbitrary operations
 * when a status has changed (e.g. any node is achieved) or at a regular frequency
 * on query set at a fixed rate of tracker_update_rate. Operations can request the
 * system to reroute and avoid certain edges. Example operations may be to:
 * reroute when blocked or at a required rate (though may want to use BTs instead),
 * adjust speed limits, wait, call an external service or action to perform a task
 * such as calling an elevator or opening an automatic door, etc.
 * Operations may throw nav2_core::OperationFailed exceptions in failure cases
 */
class RouteOperation
{
public:
  using Ptr = std::shared_ptr<nav2_route::RouteOperation>;

  /**
   * @brief Constructor
   */
  RouteOperation() = default;

  /**
   * @brief Destructor
   */
  virtual ~RouteOperation() = default;

  /**
   * @brief Configure the operation plugin (get params, create interfaces, etc)
   * @param node A node to use
   * @param name the plugin's name set by the param file that may need to be used
   * to correlate an operation instance to the navigation graph operation calls
   */
  virtual void configure(
    const nav2_util::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
    const std::string & name) = 0;

  /**
   * @brief An API to get the name of a particular operation for triggering, query
   * or logging
   * @return the plugin's name
   */
  virtual std::string getName() = 0;

  /**
   * @brief Indication of which type of route operation this plugin is. Whether it is
   * be called by the graph's nodes or edges, whether it should be triggered at any
   * status change, or whether it should be called constantly on any query. By default,
   * it will create operations that are only called when a graph's node or edge requests it.
   * Note that On Query, On Status Change, and On Graph are mutually exclusive since each
   * operation type is merely a subset of the previous level's specificity.
   * @return The type of operation (on graph call, on status changes, or constantly)
   */
  virtual RouteOperationType processType() {return RouteOperationType::ON_GRAPH;}

  /**
   * @brief The main route operation API to perform an operation when triggered.
   * The return value indicates if the route operation is requesting rerouting when returning true.
   * Could be if this operation is checking if a route is in collision or operation
   * failed (to open a door, for example) and thus this current route is now invalid.
   * @param mdata Metadata corresponding to the operation in the navigation graph.
   * If metadata is invalid or irrelevant, a nullptr is given
   * @param node_achieved Node achieved,
   * for additional context (must check nullptr if at goal)
   * @param edge_entered Edge entered by node achievement,
   * for additional context (must check if nullptr if no future edge, at goal)
   * @param edge_exited Edge exited by node achievement,
   * for additional context (must check if nullptr if no last edge, starting)
   * @param route Current route being tracked in full, for additional context
   * @param curr_pose Current robot pose in the route frame, for additional context
   * @return Whether to perform rerouting and report blocked edges in that case
   */
  virtual OperationResult perform(
    NodePtr node_achieved,
    EdgePtr edge_entered,
    EdgePtr edge_exited,
    const Route & route,
    const geometry_msgs::msg::PoseStamped & curr_pose,
    const Metadata * mdata = nullptr) = 0;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__INTERFACES__ROUTE_OPERATION_HPP_
