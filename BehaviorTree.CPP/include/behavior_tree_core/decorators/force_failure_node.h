/*  Copyright (C) 2018 Davide Faconti -  All Rights Reserved
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

#ifndef DECORATOR_ALWAYS_FAILURE_NODE_H
#define DECORATOR_ALWAYS_FAILURE_NODE_H

#include "behavior_tree_core/decorator_node.h"

namespace BT
{
class ForceFailureDecorator : public DecoratorNode
{
  public:
    ForceFailureDecorator(const std::string& name) : DecoratorNode(name, NodeParameters())
    {
    }

  private:
    virtual BT::NodeStatus tick() override;
};

//------------ implementation ----------------------------

inline NodeStatus ForceFailureDecorator::tick()
{
    setStatus(NodeStatus::RUNNING);

    const NodeStatus child_state = child_node_->executeTick();

    switch (child_state)
    {
        case NodeStatus::FAILURE:
        case NodeStatus::SUCCESS:
        {
            child_node_->setStatus(NodeStatus::IDLE);
            return NodeStatus::FAILURE;
        }

        case NodeStatus::RUNNING:
        {
            return NodeStatus::RUNNING;
        }

        default:
        {
            // TODO throw?
        }
    }
    return status();
}
}

#endif
