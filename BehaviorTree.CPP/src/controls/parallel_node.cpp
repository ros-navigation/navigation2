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

#include "behavior_tree_core/controls/parallel_node.h"

BT::ParallelNode::ParallelNode(const std::string& name, int threshold_M)
  : ControlNode::ControlNode(name, NodeParameters()), threshold_M_(threshold_M)
{
}

BT::NodeStatus BT::ParallelNode::tick()
{
    success_childred_num_ = 0;
    failure_childred_num_ = 0;
    // Vector size initialization. children_count_ could change at runtime if you edit the tree
    const unsigned children_count = children_nodes_.size();

    // Routing the tree according to the sequence node's logic:
    for (unsigned int i = 0; i < children_count; i++)
    {
        TreeNode* child_node = children_nodes_[i];

        NodeStatus child_status = child_node->executeTick();

        switch (child_status)
        {
            case NodeStatus::SUCCESS:
                child_node->setStatus(
                    NodeStatus::IDLE);   // the child goes in idle if it has returned success.
                if (++success_childred_num_ == threshold_M_)
                {
                    success_childred_num_ = 0;
                    failure_childred_num_ = 0;
                    haltChildren(0);   // halts all running children. The execution is done.
                    return child_status;
                }
                break;
            case NodeStatus::FAILURE:
                child_node->setStatus(
                    NodeStatus::IDLE);   // the child goes in idle if it has returned failure.
                if (++failure_childred_num_ > children_count - threshold_M_)
                {
                    success_childred_num_ = 0;
                    failure_childred_num_ = 0;
                    haltChildren(0);   // halts all running children. The execution is hopeless.
                    return child_status;
                }
                break;
            case NodeStatus::RUNNING:
                setStatus(child_status);
                break;
            default:
                break;
        }
    }
    return NodeStatus::RUNNING;
}

void BT::ParallelNode::halt()
{
    success_childred_num_ = 0;
    failure_childred_num_ = 0;
    BT::ControlNode::halt();
}

unsigned int BT::ParallelNode::thresholdM()
{
    return threshold_M_;
}

void BT::ParallelNode::setThresholdM(unsigned int threshold_M)
{
    threshold_M_ = threshold_M;
}
