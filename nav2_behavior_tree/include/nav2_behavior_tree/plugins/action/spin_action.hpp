// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::Spin
 */
class SpinAction : public BtActionNode<nav2_msgs::action::Spin>
{
  using Action = nav2_msgs::action::Spin;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::SpinAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  SpinAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("spin_dist", 1.57, "Spin distance"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for spinning"),
        BT::InputPort<bool>("is_recovery", true, "True if recovery"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The spin behavior error code")
      });
  }

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

private:
  bool is_recovery_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_ACTION_HPP_
