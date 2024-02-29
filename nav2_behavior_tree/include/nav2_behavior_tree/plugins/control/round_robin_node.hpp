// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__ROUND_ROBIN_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__ROUND_ROBIN_NODE_HPP_

#include <string>

#include "behaviortree_cpp/control_node.h"
#include "behaviortree_cpp/bt_factory.h"

namespace nav2_behavior_tree
{

/** @brief Type of sequence node that ticks children in a round-robin fashion
 *
 * Type of Control Node  | Child Returns Failure | Child Returns Running
 * ---------------------------------------------------------------------
 *  RoundRobin           |    Tick Next Child    | Return Running
 *
 * If the current child return failure, the next child is ticked and if the last child returns
 * failure, the first child is ticked and the cycle continues until a child returns success
 *
 * As an example, let's say this node has 3 children: A, B and C. At the start,
 * they are all IDLE.
 * |    A    |    B    |    C    |
 * --------------------------------
 * |  IDLE   |  IDLE   |  IDLE   |
 * | RUNNING |  IDLE   |  IDLE   |  - at first A gets ticked. Assume it returns RUNNING
 *                                  - RoundRobin returns RUNNING and no other nodes are ticked.
 * | FAILURE | RUNNING |  IDLE   |  - A returns FAILURE so B gets ticked and returns RUNNING
 *                                  - RoundRobin returns RUNNING and C is not ticked yet
 * | FAILURE | SUCCESS |  IDLE   |  - B returns SUCCESS, so RoundRobin halts all children and
 *                                  - returns SUCCESS, next iteration will tick C.
 * | RUNNING |  IDLE   | FAILURE |  - C returns FAILURE, so RoundRobin circles and ticks A.
 *                                  - A returns RUNNING, so RoundRobin returns RUNNING.
 *
 * If all children return FAILURE, RoundRobin will return FAILURE
 * and halt all children, ending the sequence.
 *
 * Usage in XML: <RoundRobin>
 */
class RoundRobinNode : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RoundRobinNode
   * @param name Name for the XML tag for this node
   */
  explicit RoundRobinNode(const std::string & name);

  /**
   * @brief A constructor for nav2_behavior_tree::RoundRobinNode
   * @param name Name for the XML tag for this node
   * @param config BT node configuration
   */
  RoundRobinNode(const std::string & name, const BT::NodeConfiguration & config);

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief The other (optional) override required by a BT action to reset node state
   */
  void halt() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() {return {};}

private:
  unsigned int current_child_idx_{0};
  unsigned int num_failed_children_{0};
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__ROUND_ROBIN_NODE_HPP_
