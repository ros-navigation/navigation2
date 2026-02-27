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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHECK_POSE_OCCUPANCY_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHECK_POSE_OCCUPANCY_ACTION_HPP_

#include <memory>
#include <string>

#include "nav2_ros_common/lifecycle_node.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/json_export.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/srv/get_costs.hpp"
#include "nav2_ros_common/service_client.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that checks if a pose is occupied by calling the GetCosts service on the costmap
 */
class CheckPoseOccupancy : public BtServiceNode<nav2_msgs::srv::GetCosts>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::CheckPoseOccupancy
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  CheckPoseOccupancy(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation after receiving a result from the service
   * @param response The response received from the service
   * @return BT::NodeStatus Status of tick execution after processing the response
   */
  BT::NodeStatus on_completion(std::shared_ptr<nav2_msgs::srv::GetCosts::Response> response)
  override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    // Register JSON definitions for the types used in the ports
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();

    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "Pose to check if occupied"),
        BT::InputPort<double>(
        "cost_threshold", 254.0,
        "Cost threshold for considering a pose occupied"),
        BT::InputPort<bool>("use_footprint", true, "Whether to use footprint cost"),
        BT::InputPort<bool>(
        "consider_unknown_as_obstacle", false,
        "Whether to consider unknown cost as obstacle")
    });
  }

private:
  bool use_footprint_;
  bool consider_unknown_as_obstacle_;
  double cost_threshold_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHECK_POSE_OCCUPANCY_ACTION_HPP_
