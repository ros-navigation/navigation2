// Copyright (c) 2025 Enjoy Robotics
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

#ifndef UTILS__GET_NODE_FROM_TREE_HPP_
#define UTILS__GET_NODE_FROM_TREE_HPP_

#include <memory>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief Get node from tree by type and index, casted to NodeT type, const casted away
 * Returns true if the node was found, false otherwise
 */
template<typename NodeT>
NodeT * get_node_from_tree(
  const std::shared_ptr<BT::Tree> tree,
  const size_t index = 0)
{
  static rclcpp::Logger logger = rclcpp::get_logger("nav2_behavior_tree::get_node_from_tree");

  std::vector<const BT::TreeNode *> nodes = tree->getNodesByPath<NodeT>("*");
  if (nodes.empty()) {
    RCLCPP_ERROR(logger, "No nodes of given type found");
    return nullptr;
  }
  if (nodes.size() <= index) {
    RCLCPP_ERROR(logger, "Out of bounds (found %zu < %zu nodes)", nodes.size() + 1, index);
    return nullptr;
  }

  const NodeT * const_bt_node =
    dynamic_cast<const NodeT *>(nodes[index]);
  if (const_bt_node == nullptr) {
    RCLCPP_ERROR(logger, "Failed to cast node to given type");
    return nullptr;
  }

  NodeT * bt_node = const_cast<NodeT *>(const_bt_node);
  if (bt_node == nullptr) {
    RCLCPP_ERROR(logger, "Failed to cast away const from node");
    return nullptr;
  }

  return bt_node;
}

}  // namespace nav2_behavior_tree

#endif  // UTILS__GET_NODE_FROM_TREE_HPP_
