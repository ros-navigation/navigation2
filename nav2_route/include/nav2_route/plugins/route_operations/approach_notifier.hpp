// Copyright (c) 2026 Suad Shabaj
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

#ifndef NAV2_ROUTE__PLUGINS__ROUTE_OPERATIONS__APPROACH_NOTIFIER_HPP_
#define NAV2_ROUTE__PLUGINS__ROUTE_OPERATIONS__APPROACH_NOTIFIER_HPP_

#include <memory>
#include <optional>
#include <string>

#include "rclcpp/logger.hpp"
#include "nav2_route/interfaces/route_operation.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_msgs/msg/route_node.hpp"

namespace nav2_route
{

/**
 * @class ApproachNotifier
 * @brief A route operation to notify when the robot is approaching a node ahead
 * on the route, before it is achieved, so that other systems may prepare for its arrival
 * (e.g. calling an elevator, opening a door, alerting a station). Nodes are notified
 * if they contain the `approach_tag` metadata set to true, within `approach_radius`.
 */
class ApproachNotifier : public RouteOperation
{
public:
  /**
   * @brief Constructor
   */
  ApproachNotifier() = default;

  /**
   * @brief destructor
   */
  virtual ~ApproachNotifier() = default;

  /**
   * @brief Configure
   */
  void configure(
    const nav2::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
    const std::string & name) override;

  /**
   * @brief Get name of the plugin for parameter scope mapping
   * @return Name
   */
  std::string getName() override {return name_;}

  /**
   * @brief Indication that the approach notifier route operation is performed
   * on approaching upcoming nodes
   * @return The type of operation (on graph call, on status changes, constantly, or on approach)
   */
  RouteOperationType processType() override {return RouteOperationType::ON_APPROACH;}

  /**
   * @brief The approach radius of a node, if it is tagged for approach notification
   * @param node A node ahead on the route
   * @return The approach radius, or std::nullopt if the node is not tagged
   */
  std::optional<float> approachRadius(const NodePtr node) override;

  /**
   * @brief The main approach notifier operation to publish the node being approached
   * @param node_achieved Node being approached
   * @param edge_entered Route edge leading into the node being approached,
   * for additional context
   * @param edge_exited Unused, nullptr
   * @param route Current route being tracked in full, for additional context
   * @param curr_pose Current robot pose in the route frame, for additional context
   * @param mdata Metadata of the node being approached
   * @return Whether to perform rerouting and report blocked edges in that case
   */
  OperationResult perform(
    NodePtr node_achieved,
    EdgePtr edge_entered,
    EdgePtr edge_exited,
    const Route & route,
    const geometry_msgs::msg::PoseStamped & curr_pose,
    const Metadata * mdata = nullptr) override;

protected:
  std::string name_;
  std::string approach_tag_;
  float approach_radius_{2.0f};
  rclcpp::Logger logger_{rclcpp::get_logger("ApproachNotifier")};
  nav2::Publisher<nav2_msgs::msg::RouteNode>::SharedPtr approach_pub_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__PLUGINS__ROUTE_OPERATIONS__APPROACH_NOTIFIER_HPP_
