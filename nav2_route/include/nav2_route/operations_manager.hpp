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

#include <unordered_map>
#include <vector>
#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/route_exceptions.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"
#include "nav2_route/interfaces/route_operation.hpp"

#ifndef NAV2_ROUTE__OPERATIONS_MANAGER_HPP_
#define NAV2_ROUTE__OPERATIONS_MANAGER_HPP_

namespace nav2_route
{

/**
 * @class nav2_route::OperationsManager
 * @brief Manages operations plugins to call on route tracking
 */
class OperationsManager
{
public:
  typedef std::vector<RouteOperation::Ptr>::const_iterator OperationsIter;

  /**
   * @brief A constructor for nav2_route::OperationsManager
   */
  explicit OperationsManager(
    nav2_util::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber);

  /**
   * @brief A Destructor for nav2_route::OperationsManager
   */
  ~OperationsManager() = default;

  /**
   * @brief Finds the set of operations stored in the graph to trigger at this transition
   * @param node Node to check
   * @param edge_entered Edge entered to check for ON_ENTER events
   * @param edge_exit Edge exit to check for ON_EXIT events
   * @return OperationPtrs A vector of operation pointers to execute
   */
  OperationPtrs findGraphOperations(
    const NodePtr node, const EdgePtr edge_enter, const EdgePtr edge_exit);

  /**
   * @brief Finds the set of operations stored in graph objects, by event
   * @param node op_vec Operations vector to check
   * @param trigger Trigger for which operations in op_vec should be included
   * @param operations Output vector populated with relevant operations
   */
  template<typename T>
  void findGraphOperationsToProcess(
    T & obj,
    const OperationTrigger & trigger,
    OperationPtrs & operations);

  /**
   * @brief Updates manager result state by an individual operation's result
   * @param name Operations' name
   * @param op_result Operations' result
   * @param result Manager's result to update
   */
  void updateResult(
    const std::string & name, const OperationResult & op_result, OperationsResult & result);

  /**
   * @brief Processes the operations at this tracker state
   * @param status_change Whether something meaningful has changed
   * @param state The route tracking state to check for state info
   * @param route The raw route being tracked
   * @param pose robot pose
   * @param rerouting_info Rerouting information regarding previous partial state
   * @return A result vector whether the operations are requesting something to occur
   */
  OperationsResult process(
    const bool status_change,
    const RouteTrackingState & state,
    const Route & route,
    const geometry_msgs::msg::PoseStamped & pose,
    const ReroutingState & rerouting_info);

protected:
  /**
   * @brief Processes a vector of operations plugins, by trigger
   * @param operations Operations to trigger
   * @param Results to populate from operations
   */
  void processOperationsPluginVector(
    const std::vector<RouteOperation::Ptr> & operations, OperationsResult & result,
    const NodePtr node,
    const EdgePtr edge_entered,
    const EdgePtr edge_exited,
    const Route & route,
    const geometry_msgs::msg::PoseStamped & pose);

  pluginlib::ClassLoader<RouteOperation> plugin_loader_;
  std::unordered_map<std::string, RouteOperation::Ptr> graph_operations_;
  std::vector<RouteOperation::Ptr> change_operations_;
  std::vector<RouteOperation::Ptr> query_operations_;
  rclcpp::Logger logger_{rclcpp::get_logger("OperationsManager")};
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__OPERATIONS_MANAGER_HPP_
