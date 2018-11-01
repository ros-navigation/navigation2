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

#include "behavior_tree_core/action_node.h"

namespace BT
{
ActionNodeBase::ActionNodeBase(const std::string& name, const NodeParameters& parameters)
  : LeafNode::LeafNode(name, parameters)
{
}

NodeStatus ActionNodeBase::executeTick()
{
    NodeStatus prev_status = status();

    if (prev_status == NodeStatus::IDLE || prev_status == NodeStatus::RUNNING)
    {
        setStatus(tick());
    }
    return status();
}

//-------------------------------------------------------

SimpleActionNode::SimpleActionNode(const std::string& name,
                                   SimpleActionNode::TickFunctor tick_functor)
  : ActionNodeBase(name, NodeParameters()), tick_functor_(std::move(tick_functor))
{
}

NodeStatus SimpleActionNode::tick()
{
    NodeStatus prev_status = status();

    if (prev_status == NodeStatus::IDLE)
    {
        setStatus(NodeStatus::RUNNING);
        prev_status = NodeStatus::RUNNING;
    }

    NodeStatus status = tick_functor_(*this);
    if (status != prev_status)
    {
        setStatus(status);
    }
    return status;
}

//-------------------------------------------------------

ActionNode::ActionNode(const std::string& name, const NodeParameters& parameters)
  : ActionNodeBase(name, parameters), loop_(true)
{
    thread_ = std::thread(&ActionNode::waitForTick, this);
}

ActionNode::~ActionNode()
{
    if (thread_.joinable())
    {
        stopAndJoinThread();
    }
}

void ActionNode::waitForTick()
{
    while (loop_.load())
    {
        tick_engine_.wait();

        // check loop_ again because the tick_engine_ could be
        // notified from the method stopAndJoinThread
        if (loop_ && status() == NodeStatus::IDLE)
        {
            setStatus(NodeStatus::RUNNING);
            setStatus(tick());
        }
    }
}

NodeStatus ActionNode::executeTick()
{
    //send signal to other thread.
    // The other thread is in charge for changing the status
    if (status() == NodeStatus::IDLE)
    {
        tick_engine_.notify();
    }

    // block as long as the state is NodeStatus::IDLE
    const NodeStatus stat = waitValidStatus();
    return stat;
}

void ActionNode::stopAndJoinThread()
{
    loop_.store(false);
    tick_engine_.notify();
    if (thread_.joinable())
    {
        thread_.join();
    }
}
}
