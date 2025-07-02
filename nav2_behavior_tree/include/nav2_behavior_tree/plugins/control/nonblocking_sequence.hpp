// Copyright (c) 2025 Polymath Robotics
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__NONBLOCKING_SEQUENCE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__NONBLOCKING_SEQUENCE_HPP_

#include <string>
#include "behaviortree_cpp/control_node.h"
#include "behaviortree_cpp/bt_factory.h"

namespace nav2_behavior_tree
{

/** @brief Type of sequence node that keeps tickinng through all the children until all children
 * return SUCCESS
 *
 * Type of Control Node  | Child Returns Failure | Child Returns Running
 * ---------------------------------------------------------------------
 * NonblockingSequence   |      Restart          | Continue tickng next child
 *
 * Continue ticking next child means every node after the running node will be ticked. Even
 * if a previous node returns Running, the subsequent nodes will be reticked.
 *
 * As an example, let's say this node has 3 children: A, B and C. At the start,
 * they are all IDLE.
 * |    A    |    B    |    C    |
 * --------------------------------
 * |  IDLE   |  IDLE   |   IDLE  |
 * | RUNNING |  IDLE   |   IDLE  |  - at first A gets ticked. Assume it returns RUNNING
 *                                  - NonblockingSequence returns RUNNING and continues to the next
 *                                  - node.
 * | RUNNING | RUNNING | RUNNING |  - Eventually all nodes will be in the running state and
 *                                  - NonblockingSequence returns RUNNING
 * | SUCCESS | RUNNING | SUCCESS |  - Even in a configuration where there are multiple nodes
 *                                  - returning SUCCESS, NonblockingSequence continues on ticking all.
 *                                  - nodes each time it is ticked and returns RUNNING.
 * | SUCCESS | SUCCESS | SUCCESS |  - If all child nodes return SUCCESS the NonblockingSequence
 *                                  - returns SUCCESS
 *
 * If any children at any time had returned FAILURE. NonblockingSequence would have returned FAILURE
 * and halted all children, ending the sequence.
 *
 * Usage in XML: <NonblockingSequence>
 */
class NonblockingSequence : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::NonblockingSequence
   * @param name Name for the XML tag for this node
   */
  explicit NonblockingSequence(const std::string & name);

  /**
   * @brief A constructor for nav2_behavior_tree::NonblockingSequence
   * @param name Name for the XML tag for this node
   * @param config BT node configuration
   */
  NonblockingSequence(const std::string & name, const BT::NodeConfiguration & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() {return {};}

protected:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

};
}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__NONBLOCKING_SEQUENCE_HPP_
