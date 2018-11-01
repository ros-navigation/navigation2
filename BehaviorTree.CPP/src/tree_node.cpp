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

#include "behavior_tree_core/tree_node.h"
#include <cstring>

namespace BT
{
static uint8_t getUID()
{
    static uint8_t uid = 1;
    return uid++;
}

TreeNode::TreeNode(const std::string& name, const NodeParameters& parameters)
  : name_(name), status_(NodeStatus::IDLE), uid_(getUID()), parameters_(parameters)
{
}

NodeStatus TreeNode::executeTick()
{
    const NodeStatus status = tick();
    setStatus(status);
    return status;
}

void TreeNode::setStatus(NodeStatus new_status)
{
    NodeStatus prev_status;
    {
        std::unique_lock<std::mutex> UniqueLock(state_mutex_);
        prev_status = status_;
        status_ = new_status;
    }
    if (prev_status != new_status)
    {
        state_condition_variable_.notify_all();
        state_change_signal_.notify(std::chrono::high_resolution_clock::now(), *this, prev_status,
                                    new_status);
    }
}

void TreeNode::setBlackboard(const Blackboard::Ptr& bb)
{
    bb_ = bb;
}

const Blackboard::Ptr& TreeNode::blackboard() const
{
    return bb_;
}

NodeStatus TreeNode::status() const
{
    std::lock_guard<std::mutex> LockGuard(state_mutex_);
    return status_;
}

NodeStatus TreeNode::waitValidStatus()
{
    std::unique_lock<std::mutex> lk(state_mutex_);

    state_condition_variable_.wait(lk, [&]() {
        return (status_ == NodeStatus::RUNNING || status_ == NodeStatus::SUCCESS ||
                status_ == NodeStatus::FAILURE);
    });
    return status_;
}

const std::string& TreeNode::name() const
{
    return name_;
}

bool TreeNode::isHalted() const
{
    return status() == NodeStatus::IDLE;
}

TreeNode::StatusChangeSubscriber
TreeNode::subscribeToStatusChange(TreeNode::StatusChangeCallback callback)
{
    return state_change_signal_.subscribe(std::move(callback));
}

uint16_t TreeNode::UID() const
{
    return uid_;
}

void TreeNode::setRegistrationName(const std::string& registration_name)
{
    registration_name_ = registration_name;
}

const std::string& TreeNode::registrationName() const
{
    return registration_name_;
}

const NodeParameters& TreeNode::initializationParameters() const
{
    return parameters_;
}

}   // end namespace
