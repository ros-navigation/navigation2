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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_ROUTE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_ROUTE_ACTION_HPP_

#include <string>

#include "nav2_msgs/action/compute_route.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::ComputeRoute
 */
class ComputeRouteAction : public BtActionNode<nav2_msgs::action::ComputeRoute>
{
  using Action = nav2_msgs::action::ComputeRoute;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::ComputeRoute
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  ComputeRouteAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancellation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * \brief Override required by the a BT action. Cancel the action and set the path output
   */
  void halt() override;

  /**
   * @brief Reset output port values on failure
   */
  void resetPorts();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<unsigned int>("start_id", "ID of the start node"),
        BT::InputPort<unsigned int>("goal_id", "ID of the goal node"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "start",
          "Start pose of the path if overriding current robot pose and using poses over IDs"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "goal", "Goal pose of the path if using poses over IDs"),
        BT::InputPort<bool>(
          "use_start", false,
          "Whether to use the start pose or the robot's current pose"),
        BT::InputPort<bool>(
          "use_poses", false, "Whether to use poses or IDs for start and goal"),
        BT::OutputPort<ActionResult::_route_type>(
          "route", "The route computed by ComputeRoute node"),
        BT::OutputPort<builtin_interfaces::msg::Duration>("planning_time",
          "Time taken to compute route"),
        BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputeRoute node"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The compute route error code"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_ROUTE_ACTION_HPP_
