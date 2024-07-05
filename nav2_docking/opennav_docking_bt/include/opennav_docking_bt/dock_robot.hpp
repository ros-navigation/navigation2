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

#ifndef OPENNAV_DOCKING_BT__DOCK_ROBOT_HPP_
#define OPENNAV_DOCKING_BT__DOCK_ROBOT_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_msgs/action/dock_robot.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace opennav_docking_bt
{

/**
 * @brief nav2_behavior_tree::BtActionNode class that wraps opnav2_msgsennav_docking_msgs/DockRobot
 */
class DockRobotAction
  : public nav2_behavior_tree::BtActionNode<
    nav2_msgs::action::DockRobot>
{
  using Action = nav2_msgs::action::DockRobot;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for opennav_docking_bt::DockRobotAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  DockRobotAction(
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
        BT::InputPort<bool>(
          "use_dock_id", true,
          "Whether to use the dock's ID or dock pose fields"),
        BT::InputPort<std::string>("dock_id", "Dock ID or name to use"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "dock_pose", "The dock pose, if not using dock id"),
        BT::InputPort<std::string>("dock_type", "The dock plugin type, if using dock pose"),
        BT::InputPort<float>(
          "max_staging_time", 1000.0, "Maximum time to navigate to the staging pose"),
        BT::InputPort<bool>(
          "navigate_to_staging_pose", true, "Whether to autonomously navigate to staging pose"),

        BT::OutputPort<ActionResult::_success_type>(
          "success", "If the action was successful"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "Error code"),
        BT::OutputPort<ActionResult::_num_retries_type>(
          "num_retries", "The number of retries executed"),
      });
  }
};

}  // namespace opennav_docking_bt

#endif  // OPENNAV_DOCKING_BT__DOCK_ROBOT_HPP_
