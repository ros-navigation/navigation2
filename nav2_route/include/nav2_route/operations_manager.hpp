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

#include <unordered_map>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
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
  typedef std::vector<RouteOperation::Ptr>::iterator OperationsIter;

  /**
   * @brief A constructor for nav2_route::OperationsManager
   */
  explicit OperationsManager(nav2_util::LifecycleNode::SharedPtr node);

  /**
   * @brief A Destructor for nav2_route::OperationsManager
   */
  ~OperationsManager() = default;

  OperationsPtr findGraphOperationsToProcess(
    const NodePtr node, const EdgePtr edge_enter, const EdgePtr edge_exit);

  OperationsResult process(
    const bool status_change,
    const RouteTrackingState & state,
    const Route & route,
    const geometry_msgs::msg::PoseStamped & pose);

protected:
  pluginlib::ClassLoader<RouteOperation> plugin_loader_;
  std::unordered_map<std::string, RouteOperation::Ptr> graph_operations_;
  std::vector<RouteOperation::Ptr> change_operations_;
  std::vector<RouteOperation::Ptr> query_operations_;
  rclcpp::Logger logger_{rclcpp::get_logger("OperationsManager")};
  bool use_feedback_operations_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__OPERATIONS_MANAGER_HPP_
