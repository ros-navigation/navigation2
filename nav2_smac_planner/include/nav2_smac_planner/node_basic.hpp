// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef NAV2_SMAC_PLANNER__NODE_BASIC_HPP_
#define NAV2_SMAC_PLANNER__NODE_BASIC_HPP_

#include <type_traits>

#include "nav2_smac_planner/constants.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "nav2_smac_planner/node_2d.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/collision_checker.hpp"

namespace nav2_smac_planner
{

/**
 * @class nav2_smac_planner::NodeBasic
 * @brief NodeBasic implementation for priority queue insertion
 */
template<typename NodeT>
class NodeBasic
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::NodeBasic
   * @param index The index of this node for self-reference
   */
  explicit NodeBasic(const uint64_t new_index)
  : graph_node_ptr(nullptr),
    index(new_index)
  {
  }

  /**
   * @brief Take a NodeBasic and populate it with any necessary state
   * cached in the queue for NodeT.
   * @param node NodeT ptr to populate metadata into NodeBasic
   */
  void populateSearchNode(NodeT * & node)
  {
    if constexpr (std::is_base_of_v<Node2D, NodeT> && !std::is_base_of_v<NodeHybrid, NodeT>) {
      // Node2D or derived: only set graph_node_ptr
      this->graph_node_ptr = node;
    } else if constexpr (std::is_base_of_v<NodeLattice, NodeT>) {
      // NodeLattice or derived: cache pose and primitive
      this->pose = node->pose;
      this->graph_node_ptr = node;
      this->prim_ptr = node->getMotionPrimitive();
      this->backward = node->isBackward();
    } else if constexpr (std::is_base_of_v<NodeHybrid, NodeT>) {
      // NodeHybrid or derived: cache pose and motion info
      this->pose = node->pose;
      this->graph_node_ptr = node;
      this->motion_index = node->getMotionPrimitiveIndex();
      this->turn_dir = node->getTurnDirection();
    } else {
      // Unknown node type - set basics
      this->graph_node_ptr = node;
    }
  }

  /**
   * @brief Take a NodeBasic and populate it with any necessary state
   * cached in the queue for NodeTs.
   * @param node Search node (basic) object to initialize internal node
   * with state
   */
  void processSearchNode()
  {
    if constexpr (std::is_base_of_v<Node2D, NodeT> && !std::is_base_of_v<NodeHybrid, NodeT>) {
      // Node2D or derived: no-op
    } else if constexpr (std::is_base_of_v<NodeLattice, NodeT>) {
      // NodeLattice or derived: update pose and motion primitive
      // We only want to override the node's pose/primitive if it has not yet been visited
      // to prevent the case that a node has been queued multiple times and
      // a new branch is overriding one of lower cost already visited.
      if (!this->graph_node_ptr->wasVisited()) {
        this->graph_node_ptr->pose = this->pose;
        this->graph_node_ptr->setMotionPrimitive(this->prim_ptr);
        this->graph_node_ptr->backwards(this->backward);
      }
    } else if constexpr (std::is_base_of_v<NodeHybrid, NodeT>) {
      // NodeHybrid or derived: update pose and motion primitive index
      // We only want to override the node's pose if it has not yet been visited
      // to prevent the case that a node has been queued multiple times and
      // a new branch is overriding one of lower cost already visited.
      if (!this->graph_node_ptr->wasVisited()) {
        this->graph_node_ptr->pose = this->pose;
        this->graph_node_ptr->setMotionPrimitiveIndex(this->motion_index, this->turn_dir);
      }
    } else {
      // Unknown node type: no-op
    }
  }

  typename NodeT::Coordinates pose;  // Used by NodeHybrid and NodeLattice
  NodeT * graph_node_ptr;
  MotionPrimitive * prim_ptr;  // Used by NodeLattice
  uint64_t index;
  unsigned int motion_index;
  bool backward;
  TurnDirection turn_dir;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__NODE_BASIC_HPP_
