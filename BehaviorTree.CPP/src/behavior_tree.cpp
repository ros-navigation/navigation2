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

#include "behavior_tree_core/behavior_tree.h"
#include <cstring>

namespace BT
{
void applyRecursiveVisitor(const TreeNode* node,
                           const std::function<void(const TreeNode*)>& visitor)
{
    if (!node)
    {
        throw std::runtime_error("One of the children of a DecoratorNode or ControlNode is nulltr");
    }

    visitor(node);

    if (auto control = dynamic_cast<const BT::ControlNode*>(node))
    {
        for (const auto& child : control->children())
        {
            applyRecursiveVisitor(static_cast<const TreeNode*>(child), visitor);
        }
    }
    else if (auto decorator = dynamic_cast<const BT::DecoratorNode*>(node))
    {
        applyRecursiveVisitor(decorator->child(), visitor);
    }
}

void applyRecursiveVisitor(TreeNode* node, const std::function<void(TreeNode*)>& visitor)
{
    if (!node)
    {
        throw std::runtime_error("One of the children of a DecoratorNode or ControlNode is nulltr");
    }

    visitor(node);

    if (auto control = dynamic_cast<BT::ControlNode*>(node))
    {
        for (const auto& child : control->children())
        {
            applyRecursiveVisitor(child, visitor);
        }
    }
    else if (auto decorator = dynamic_cast<BT::DecoratorNode*>(node))
    {
        applyRecursiveVisitor(decorator->child(), visitor);
    }
}

void printTreeRecursively(const TreeNode* root_node)
{
    std::function<void(int, const BT::TreeNode*)> recursivePrint;

    recursivePrint = [&recursivePrint](unsigned indent, const BT::TreeNode* node) {
        for (unsigned i = 0; i < indent; i++)
        {
            std::cout << "   ";
        }
        if (!node)
        {
            std::cout << "!nullptr!" << std::endl;
            return;
        }
        std::cout << node->name() << std::endl;
        indent++;

        if (auto control = dynamic_cast<const BT::ControlNode*>(node))
        {
            for (const auto& child : control->children())
            {
                recursivePrint(indent, child);
            }
        }
        else if (auto decorator = dynamic_cast<const BT::DecoratorNode*>(node))
        {
            recursivePrint(indent, decorator->child());
        }
    };

    std::cout << "----------------" << std::endl;
    recursivePrint(0, root_node);
    std::cout << "----------------" << std::endl;
}

void buildSerializedStatusSnapshot(TreeNode* root_node, SerializedTreeStatus& serialized_buffer)
{
    serialized_buffer.clear();

    auto visitor = [&serialized_buffer](const TreeNode* node) {
        serialized_buffer.push_back(
            std::make_pair(node->UID(), static_cast<uint8_t>(node->status())));
    };

    applyRecursiveVisitor(root_node, visitor);
}

void assignBlackboardToEntireTree(TreeNode* root_node, const Blackboard::Ptr& bb)
{
    auto visitor = [bb](TreeNode* node) { node->setBlackboard(bb); };
    applyRecursiveVisitor(root_node, visitor);
}

void haltAllActions(TreeNode* root_node)
{
    auto visitor = [](TreeNode* node) {
        if (auto action = dynamic_cast<ActionNode*>(node))
        {
            action->stopAndJoinThread();
        }
    };
    applyRecursiveVisitor(root_node, visitor);
}
}
