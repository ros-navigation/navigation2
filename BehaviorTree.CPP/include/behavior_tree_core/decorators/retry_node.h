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

#ifndef DECORATORRETRYNODE_H
#define DECORATORRETRYNODE_H

#include "behavior_tree_core/decorator_node.h"

namespace BT
{
class RetryNode : public DecoratorNode
{
  public:
    // Constructor
    RetryNode(const std::string& name, unsigned int NTries);

    RetryNode(const std::string& name, const NodeParameters& params);

    virtual ~RetryNode() override = default;

    static const NodeParameters& requiredNodeParameters()
    {
        static NodeParameters params = {{NUM_ATTEMPTS, "1"}};
        return params;
    }

  private:
    unsigned int NTries_;
    unsigned int TryIndx_;

    static constexpr const char* NUM_ATTEMPTS = "num_attempts";
    virtual BT::NodeStatus tick() override;
};
}

#endif
