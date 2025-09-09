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

#ifndef NAV2_ROUTE__PLUGINS__ROUTE_OPERATIONS__REROUTING_SERVICE_HPP_
#define NAV2_ROUTE__PLUGINS__ROUTE_OPERATIONS__REROUTING_SERVICE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_route/interfaces/route_operation.hpp"
#include "nav2_util/node_utils.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace nav2_route
{

/**
 * @class ReroutingService
 * @brief A route operation to process requests from an external server for rerouting
 */
class ReroutingService : public RouteOperation
{
public:
  /**
   * @brief Constructor
   */
  ReroutingService() = default;

  /**
   * @brief destructor
   */
  virtual ~ReroutingService() = default;

  /**
   * @brief Configure
   */
  void configure(
    const nav2_util::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
    const std::string & name) override;

  /**
   * @brief Get name of the plugin for parameter scope mapping
   * @return Name
   */
  std::string getName() override {return name_;}

  /**
   * @brief Indication that the adjust speed limit route operation is performed
   * on all state changes
   * @return The type of operation (on graph call, on status changes, or constantly)
   */
  RouteOperationType processType() override {return RouteOperationType::ON_QUERY;}

  /**
   * @brief The main speed limit operation to adjust the maximum speed of the vehicle
   * @param mdata Metadata corresponding to the operation in the navigation graph.
   * If metadata is invalid or irrelevant, a nullptr is given
   * @param node_achieved Node achieved, for additional context
   * @param edge_entered Edge entered by node achievement, for additional context
   * @param edge_exited Edge exited by node achievement, for additional context
   * @param route Current route being tracked in full, for additional context
   * @param curr_pose Current robot pose in the route frame, for additional context
   * @return Whether to perform rerouting and report blocked edges in that case
   */
  OperationResult perform(
    NodePtr node,
    EdgePtr edge_entered,
    EdgePtr edge_exited,
    const Route & route,
    const geometry_msgs::msg::PoseStamped & curr_pose,
    const Metadata * mdata = nullptr) override;

  /**
   * @brief Service callback to trigger a reroute externally
   * @param request, empty
   * @param response, returns success
   */
  void serviceCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

protected:
  std::string name_;
  std::atomic_bool reroute_;
  rclcpp::Logger logger_{rclcpp::get_logger("ReroutingService")};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__PLUGINS__ROUTE_OPERATIONS__REROUTING_SERVICE_HPP_
