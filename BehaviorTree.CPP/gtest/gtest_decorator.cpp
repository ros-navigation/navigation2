/* Copyright (C) 2018 Davide Faconti - All Rights Reserved
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

#include <gtest/gtest.h>
#include "action_test_node.h"
#include "condition_test_node.h"
#include "behavior_tree_core/behavior_tree.h"

using BT::NodeStatus;

struct DeadlineTest : testing::Test
{
    BT::TimeoutNode root;
    BT::AsyncActionTest action;

    DeadlineTest() : root("deadline", 250), action("action")
    {
        root.setChild(&action);
    }
    ~DeadlineTest()
    {
        haltAllActions(&root);
    }
};

/****************TESTS START HERE***************************/

TEST_F(DeadlineTest, DeadlineTriggeredTest)
{
    BT::NodeStatus state = root.executeTick();
    // deadline in 250 ms
    action.setTime(3);

    ASSERT_EQ(NodeStatus::RUNNING, action.status());
    ASSERT_EQ(NodeStatus::RUNNING, state);

    std::this_thread::sleep_for(std::chrono::milliseconds(350));
    state = root.executeTick();
    ASSERT_EQ(NodeStatus::IDLE, action.status());
    ASSERT_EQ(NodeStatus::FAILURE, state);
}

TEST_F(DeadlineTest, DeadlineNotTriggeredTest)
{
    BT::NodeStatus state = root.executeTick();
    // deadline in 250 ms
    action.setTime(2);

    ASSERT_EQ(NodeStatus::RUNNING, action.status());
    ASSERT_EQ(NodeStatus::RUNNING, state);

    std::this_thread::sleep_for(std::chrono::milliseconds(350));
    state = root.executeTick();
    ASSERT_EQ(NodeStatus::IDLE, action.status());
    ASSERT_EQ(NodeStatus::SUCCESS, state);
}
