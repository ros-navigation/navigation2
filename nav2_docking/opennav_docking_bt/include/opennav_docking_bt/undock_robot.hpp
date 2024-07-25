// Copyright (c) 2024 Open Navigation LLC
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

#ifndef OPENNAV_DOCKING_BT__UNDOCK_ROBOT_HPP_
#define OPENNAV_DOCKING_BT__UNDOCK_ROBOT_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_msgs/action/undock_robot.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace opennav_docking_bt
{

/**
 * @brief nav2_behavior_tree::BtActionNode class that wraps nav2_msgs/UndockRobot
 */
class UndockRobotAction
  : public nav2_behavior_tree::BtActionNode<
    nav2_msgs::action::UndockRobot>
{
  using Action = nav2_msgs::action::UndockRobot;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for opennav_docking_bt::UndockRobotAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  UndockRobotAction(
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
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>(
          "dock_type", "The dock plugin type, if not previous instance used for docking"),
        BT::InputPort<float>(
          "max_undocking_time", 30.0, "Maximum time to get back to the staging pose"),

        BT::OutputPort<ActionResult::_success_type>(
          "success", "If the action was successful"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "Error code"),
      });
  }
};

}  // namespace opennav_docking_bt

#endif  // OPENNAV_DOCKING_BT__UNDOCK_ROBOT_HPP_
