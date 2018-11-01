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

#ifndef CONTROLNODE_H
#define CONTROLNODE_H

#include <vector>
#include "behavior_tree_core/tree_node.h"

namespace BT
{
class ControlNode : public TreeNode
{
  protected:
    // Children vector
    std::vector<TreeNode*> children_nodes_;

  public:
    // Constructor
    ControlNode(const std::string& name, const NodeParameters& parameters);

    virtual ~ControlNode() override = default;

    // The method used to fill the child vector
    void addChild(TreeNode* child);

    // The method used to know the number of children
    unsigned int childrenCount() const;

    const std::vector<TreeNode*>& children() const;

    const TreeNode* child(unsigned index) const
    {
        return children().at(index);
    }

    // The method used to interrupt the execution of the node
    virtual void halt() override;

    void haltChildren(int i);

    virtual NodeType type() const override final
    {
        return NodeType::CONTROL;
    }
};
}

#endif
