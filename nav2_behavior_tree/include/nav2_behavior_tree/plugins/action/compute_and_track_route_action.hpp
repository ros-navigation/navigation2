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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_AND_TRACK_ROUTE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_AND_TRACK_ROUTE_ACTION_HPP_

#include <string>
#include <memory>

#include "nav2_msgs/action/compute_and_track_route.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::ComputeAndTrackRoute
 */
class ComputeAndTrackRouteAction : public BtActionNode<nav2_msgs::action::ComputeAndTrackRoute>
{
  using Action = nav2_msgs::action::ComputeAndTrackRoute;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::ComputeAndTrackRouteAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  ComputeAndTrackRouteAction(
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
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet
   * @param feedback shared_ptr to latest feedback message
   */
  void on_wait_for_result(
    std::shared_ptr<const Action::Feedback> feedback) override;

  /**
   * @brief Function to set all feedbacks and output ports to be null values
   */
  void resetFeedbackAndOutputPorts();

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
        BT::OutputPort<builtin_interfaces::msg::Duration>(
          "execution_duration",
          "Time taken to compute and track route"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The compute route error code"),
        BT::OutputPort<uint16_t>(
          "last_node_id",
          "ID of the previous node"),
        BT::OutputPort<uint16_t>(
          "next_node_id",
          "ID of the next node"),
        BT::OutputPort<uint16_t>(
          "current_edge_id",
          "ID of current edge"),
        BT::OutputPort<nav2_msgs::msg::Route>(
          "route",
          "List of RouteNodes to go from start to end"),
        BT::OutputPort<nav_msgs::msg::Path>(
          "path",
          "Path created by ComputeAndTrackRoute node"),
        BT::OutputPort<bool>(
          "rerouted",
          "Whether the plan has rerouted"),
      });
  }

protected:
  Action::Feedback feedback_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_AND_TRACK_ROUTE_ACTION_HPP_
