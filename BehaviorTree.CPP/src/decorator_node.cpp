/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
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

#include "behavior_tree_core/decorator_node.h"

namespace BT
{
DecoratorNode::DecoratorNode(const std::string& name, const NodeParameters& parameters)
  : TreeNode::TreeNode(name, parameters), child_node_(nullptr)
{
    // TODO(...) In case it is desired to set to idle remove the ReturnStatus
    // type in order to set the member variable
    // ReturnStatus const NodeStatus child_status = NodeStatus::IDLE;  // commented out as unused
}

void DecoratorNode::setChild(TreeNode* child)
{
    if (child_node_)
    {
        throw BehaviorTreeException("Decorator '" + name() + "' has already a child assigned");
    }

    child_node_ = child;
}

void DecoratorNode::halt()
{
    haltChild();
    setStatus(NodeStatus::IDLE);
}

const TreeNode* DecoratorNode::child() const
{
    return child_node_;
}

TreeNode* DecoratorNode::child()
{
    return child_node_;
}

void DecoratorNode::haltChild()
{
    if (child_node_->status() == NodeStatus::RUNNING)
    {
        child_node_->halt();
    }
}

SimpleDecoratorNode::SimpleDecoratorNode(const std::string& name, TickFunctor tick_functor)
  : DecoratorNode(name, NodeParameters()), tick_functor_(std::move(tick_functor))
{
}

NodeStatus SimpleDecoratorNode::tick()
{
    return tick_functor_(child()->executeTick(), *this);
}
}
