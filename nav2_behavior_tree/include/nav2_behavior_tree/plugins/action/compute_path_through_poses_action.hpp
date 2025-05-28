// Copyright (c) 2021 Samsung Research America
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_

#include <string>
#include <vector>

#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav_msgs/msg/path.h"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{


/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::ComputePathThroughPoses
 * @note This is an Asynchronous (long-running) node which may return a RUNNING state while executing.
 *       It will re-initialize when halted.
 */
class ComputePathThroughPosesAction
  : public BtActionNode<nav2_msgs::action::ComputePathThroughPoses>
{
  using Action = nav2_msgs::action::ComputePathThroughPoses;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::ComputePathThroughPosesAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  ComputePathThroughPosesAction(
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
   * @brief Function to perform work in a BT Node when the action server times out
   * Such as setting the error code ID status to timed out for action clients.
   */
  void on_timeout() override;

  /**
   * \brief Override required by the a BT action. Cancel the action and set the path output
   */
  void halt() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    // Register JSON definitions for the types used in the ports
    BT::RegisterJsonDefinition<nav_msgs::msg::Path>();
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();

    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Goals>(
          "goals",
          "Destinations to plan through"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "start", "Start pose of the path if overriding current robot pose"),
        BT::InputPort<std::string>(
          "planner_id", "",
          "Mapped name to the planner plugin type to use"),
        BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathThroughPoses node"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The compute path through poses error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The compute path through poses error msg"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_
