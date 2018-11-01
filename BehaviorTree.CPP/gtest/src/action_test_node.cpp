/* Copyright (C) 2015-2017 Michele Colledanchise -  All Rights Reserved
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

#include "action_test_node.h"
#include <string>

BT::AsyncActionTest::AsyncActionTest(const std::string& name) : ActionNode(name)
{
    boolean_value_ = true;
    time_ = 3;
    stop_loop_ = false;
    tick_count_ = 0;
}

BT::AsyncActionTest::~AsyncActionTest()
{
    halt();
}

BT::NodeStatus BT::AsyncActionTest::tick()
{
    tick_count_++;
    stop_loop_ = false;
    int i = 0;
    while (!stop_loop_ && i++ < time_)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!stop_loop_)
    {
        return boolean_value_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
    else
    {
        return NodeStatus::IDLE;
    }
}

void BT::AsyncActionTest::halt()
{
    stop_loop_ = true;
    setStatus(NodeStatus::IDLE);
}

void BT::AsyncActionTest::setTime(int time)
{
    time_ = time;
}

void BT::AsyncActionTest::setBoolean(bool boolean_value)
{
    boolean_value_ = boolean_value;
}

//----------------------------------------------

BT::SyncActionTest::SyncActionTest(const std::string& name) : ActionNodeBase(name)
{
    tick_count_ = 0;
    boolean_value_ = true;
}

BT::NodeStatus BT::SyncActionTest::tick()
{
    tick_count_++;
    return boolean_value_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

void BT::SyncActionTest::setBoolean(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
