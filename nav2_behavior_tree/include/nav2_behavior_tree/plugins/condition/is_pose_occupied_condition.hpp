// Copyright (c) 2025 Maurice Alexander Purnawan
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_POSE_OCCUPIED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_POSE_OCCUPIED_CONDITION_HPP_

#include <string>
#include <memory>

#include "nav2_ros_common/lifecycle_node.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/json_export.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/srv/get_costs.hpp"
#include "nav2_ros_common/service_client.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"


namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when the IsPoseOccupied
 * service returns true and FAILURE otherwise
 */
class IsPoseOccupiedCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsPoseOccupiedCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsPoseOccupiedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsPoseOccupiedCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    // Register JSON definitions for the types used in the ports
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
    BT::RegisterJsonDefinition<std::chrono::milliseconds>();

    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "Pose to check if occupied"),
      BT::InputPort<std::string>("service_name", "global_costmap/get_cost_global_costmap"),
      BT::InputPort<double>(
          "cost_threshold", 254.0,
          "Cost threshold for considering a pose occupied"),
      BT::InputPort<bool>("use_footprint", true, "Whether to use footprint cost"),
      BT::InputPort<bool>(
          "consider_unknown_as_obstacle", false,
          "Whether to consider unknown cost as obstacle"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout"),
    };
  }

private:
  nav2::LifecycleNode::SharedPtr node_;
  nav2::ServiceClient<nav2_msgs::srv::GetCosts>::SharedPtr client_;
  // The timeout value while waiting for a response from the
  // get cost service
  std::chrono::milliseconds server_timeout_;
  bool use_footprint_;
  bool consider_unknown_as_obstacle_;
  double cost_threshold_;
  std::string service_name_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_POSE_OCCUPIED_CONDITION_HPP_
