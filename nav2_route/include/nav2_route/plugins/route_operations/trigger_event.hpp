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

#ifndef NAV2_ROUTE__PLUGINS__ROUTE_OPERATIONS__TRIGGER_EVENT_HPP_
#define NAV2_ROUTE__PLUGINS__ROUTE_OPERATIONS__TRIGGER_EVENT_HPP_

#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_route/interfaces/route_operation.hpp"
#include "nav2_core/route_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav2_route/plugins/route_operation_client.hpp"

namespace nav2_route
{

/**
 * @class TriggerEvent
 * @brief A route operation to trigger an event at a node or edge. This operation is meant to be
 * named accordingly to the event of query in the parameter file (e.g. OpenDoor, CallElevator).
 * Thus, a single TriggerEvent plugin type can support many different operation instances
 * calling different services. It may be set up to either trigger a
 * single service if the `service_name` is set in the parameter file at launch
 * **or** trigger different services depending on the `service_name` set in the metadata of the
 * node or edge operation given for centralized or decentralized events by node or edge.
 *
 * See the Route Operation Client for more details
 */
class TriggerEvent : public RouteOperationClient<std_srvs::srv::Trigger>
{
public:
  /**
   * @brief Constructor
   */
  TriggerEvent() = default;

  /**
   * @brief destructor
   */
  virtual ~TriggerEvent() = default;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__PLUGINS__ROUTE_OPERATIONS__TRIGGER_EVENT_HPP_
