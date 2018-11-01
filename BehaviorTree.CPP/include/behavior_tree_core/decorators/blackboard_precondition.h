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

#ifndef DECORATOR_BLACKBOARD_PRECONDITION_NODE_H
#define DECORATOR_BLACKBOARD_PRECONDITION_NODE_H

#include "behavior_tree_core/decorator_node.h"

namespace BT
{
template <typename T>
class BlackboardPreconditionNode : public DecoratorNode
{
  public:
    BlackboardPreconditionNode(const std::string& name, const NodeParameters& params)
      : DecoratorNode(name, params)
    {
    }

    virtual ~BlackboardPreconditionNode() override = default;

    static const NodeParameters& requiredNodeParameters()
    {
        static NodeParameters params = {{"key", ""}, {"expected", "*"}};
        return params;
    }

  private:
    virtual BT::NodeStatus tick() override
    {
        std::string key;
        T expected;
        T value;

        setStatus(NodeStatus::RUNNING);

        if (blackboard() &&                     //blackboard not null
            getParam("key", key) &&             // parameter key provided
            getParam("expected", expected) &&   // parameter expected provided
            blackboard()->get(key, value) &&    // value found in blackboard
            (value == expected ||
             initializationParameters().at("expected") == "*"))   // is expected value or "*"
        {
            return child_node_->executeTick();
        }
        else
        {
            return NodeStatus::FAILURE;
        }
    }
};
}

#endif
