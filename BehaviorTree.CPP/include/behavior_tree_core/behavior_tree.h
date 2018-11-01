/* Copyright (C) 2015-2018 Michele Colledanchise -  All Rights Reserved
 * Copyright (C) 2018 Davide Faconti -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef BEHAVIOR_TREE_H
#define BEHAVIOR_TREE_H

#include "behavior_tree_core/controls/parallel_node.h"
#include "behavior_tree_core/controls/fallback_node.h"
#include "behavior_tree_core/controls/sequence_node.h"

#include "behavior_tree_core/controls/sequence_star_node.h"
#include "behavior_tree_core/controls/fallback_star_node.h"

#include "behavior_tree_core/action_node.h"
#include "behavior_tree_core/condition_node.h"

#include "behavior_tree_core/decorators/inverter_node.h"
#include "behavior_tree_core/decorators/retry_node.h"
#include "behavior_tree_core/decorators/repeat_node.h"
#include "behavior_tree_core/decorators/subtree_node.h"

#include "behavior_tree_core/actions/always_success_node.h"
#include "behavior_tree_core/actions/always_failure_node.h"
#include "behavior_tree_core/actions/set_blackboard_node.h"

#include "behavior_tree_core/decorators/force_success_node.h"
#include "behavior_tree_core/decorators/force_failure_node.h"
#include "behavior_tree_core/decorators/blackboard_precondition.h"
#include "behavior_tree_core/decorators/timeout_node.h"

namespace BT
{
void applyRecursiveVisitor(const TreeNode* root_node,
                           const std::function<void(const TreeNode*)>& visitor);

void applyRecursiveVisitor(TreeNode* root_node, const std::function<void(TreeNode*)>& visitor);

/**
 * Debug function to print on a stream
 */
void printTreeRecursively(const TreeNode* root_node);

void assignBlackboardToEntireTree(TreeNode* root_node, const Blackboard::Ptr& bb);

void haltAllActions(TreeNode* root_node);

typedef std::vector<std::pair<uint16_t, uint8_t>> SerializedTreeStatus;

/**
 * @brief buildSerializedStatusSnapshot can be used to create a serialize buffer that can be stored
 * (or sent to a client application) to know the status of all the nodes of a tree.
 * It is not "human readable".
 *
 * @param root_node
 * @param serialized_buffer is the output.
 */
void buildSerializedStatusSnapshot(const TreeNode* root_node,
                                   SerializedTreeStatus& serialized_buffer);

/// Simple way to extract the type of a TreeNode at COMPILE TIME.
/// Useful to avoid the cost of without dynamic_cast or the virtual method TreeNode::type().
template <typename T>
inline NodeType getType()
{
    // clang-format off
    if( std::is_base_of<ActionNodeBase, T>::value )        return NodeType::ACTION;
    if( std::is_base_of<ConditionNode, T>::value )         return NodeType::CONDITION;
    if( std::is_base_of<DecoratorSubtreeNode, T>::value )  return NodeType::SUBTREE;
    if( std::is_base_of<DecoratorNode, T>::value )         return NodeType::DECORATOR;
    if( std::is_base_of<ControlNode, T>::value )           return NodeType::CONTROL;
    return NodeType::UNDEFINED;
    // clang-format on
}
}

#endif   // BEHAVIOR_TREE_H
