/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
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

struct SimpleParallelTest : testing::Test
{
    BT::ParallelNode root;
    BT::AsyncActionTest action_1;
    BT::ConditionTestNode condition_1;

    BT::AsyncActionTest action_2;
    BT::ConditionTestNode condition_2;

    SimpleParallelTest()
      : root("root_parallel", 4)
      , action_1("action_1")
      , condition_1("condition_1")
      , action_2("action_2")
      , condition_2("condition_2")
    {
        root.addChild(&condition_1);
        root.addChild(&action_1);
        root.addChild(&condition_2);
        root.addChild(&action_2);
    }
    ~SimpleParallelTest()
    {
        haltAllActions(&root);
    }
};

struct ComplexParallelTest : testing::Test
{
    BT::ParallelNode root;
    BT::ParallelNode parallel_1;
    BT::ParallelNode parallel_2;

    BT::AsyncActionTest action_1;
    BT::ConditionTestNode condition_1;

    BT::AsyncActionTest action_2;
    BT::ConditionTestNode condition_2;

    BT::AsyncActionTest action_3;
    BT::ConditionTestNode condition_3;

    ComplexParallelTest()
      : root("root", 2)
      , parallel_1("par1", 3)
      , parallel_2("par2", 1)
      , action_1("action_1")
      , condition_1("condition_1")
      , action_2("action_2")
      , condition_2("condition_2")
      , action_3("action_3")
      , condition_3("condition_3")
    {
        root.addChild(&parallel_1);
        {
            parallel_1.addChild(&condition_1);
            parallel_1.addChild(&action_1);
            parallel_1.addChild(&condition_2);
            parallel_1.addChild(&action_2);
        }
        root.addChild(&parallel_2);
        {
            parallel_2.addChild(&condition_3);
            parallel_2.addChild(&action_3);
        }
    }
    ~ComplexParallelTest()
    {
        haltAllActions(&root);
    }
};

/****************TESTS START HERE***************************/

TEST_F(SimpleParallelTest, ConditionsTrue)
{
    BT::NodeStatus state = root.executeTick();

    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::RUNNING, action_1.status());
    ASSERT_EQ(NodeStatus::RUNNING, action_2.status());
    ASSERT_EQ(NodeStatus::RUNNING, state);
}

TEST_F(SimpleParallelTest, Threshold_3)
{
    root.setThresholdM(3);
    action_2.setTime(200);
    root.executeTick();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    BT::NodeStatus state = root.executeTick();

    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::IDLE, action_1.status());
    ASSERT_EQ(NodeStatus::IDLE, action_2.status());
    ASSERT_EQ(NodeStatus::SUCCESS, state);
}

TEST_F(SimpleParallelTest, Threshold_1)
{
    root.setThresholdM(1);
    BT::NodeStatus state = root.executeTick();

    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::IDLE, action_1.status());
    ASSERT_EQ(NodeStatus::IDLE, action_2.status());
    ASSERT_EQ(NodeStatus::SUCCESS, state);
}
TEST_F(ComplexParallelTest, ConditionsTrue)
{
    BT::NodeStatus state = root.executeTick();

    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_3.status());
    ASSERT_EQ(NodeStatus::RUNNING, action_1.status());
    ASSERT_EQ(NodeStatus::RUNNING, action_2.status());
    ASSERT_EQ(NodeStatus::IDLE, action_3.status());
    ASSERT_EQ(NodeStatus::RUNNING, parallel_1.status());
    ASSERT_EQ(NodeStatus::IDLE, parallel_2.status());
    ASSERT_EQ(NodeStatus::RUNNING, state);
}

TEST_F(ComplexParallelTest, Condition3False)
{
    condition_3.setBoolean(false);
    BT::NodeStatus state = root.executeTick();

    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_3.status());
    ASSERT_EQ(NodeStatus::RUNNING, action_1.status());
    ASSERT_EQ(NodeStatus::RUNNING, action_2.status());
    ASSERT_EQ(NodeStatus::RUNNING, action_3.status());
    ASSERT_EQ(NodeStatus::RUNNING, parallel_1.status());
    ASSERT_EQ(NodeStatus::RUNNING, parallel_2.status());
    ASSERT_EQ(NodeStatus::RUNNING, state);
}

TEST_F(ComplexParallelTest, Condition3FalseAction1Done)
{
    action_2.setTime(10);
    action_3.setTime(10);

    condition_3.setBoolean(false);
    BT::NodeStatus state = root.executeTick();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_3.status());
    ASSERT_EQ(NodeStatus::SUCCESS,
              action_1.status());   // success not read yet by the node parallel_1
    ASSERT_EQ(NodeStatus::RUNNING,
              parallel_1.status());   // parallel_1 hasn't realize (yet) that action_1 has succeeded

    state = root.executeTick();

    ASSERT_EQ(NodeStatus::IDLE, action_1.status());
    ASSERT_EQ(NodeStatus::IDLE, parallel_1.status());
    ASSERT_EQ(NodeStatus::IDLE, action_2.status());
    ASSERT_EQ(NodeStatus::RUNNING, action_3.status());
    ASSERT_EQ(NodeStatus::RUNNING, parallel_2.status());
    ASSERT_EQ(NodeStatus::RUNNING, state);

    state = root.executeTick();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    state = root.executeTick();

    ASSERT_EQ(NodeStatus::IDLE, parallel_2.status());
    ASSERT_EQ(NodeStatus::IDLE, action_1.status());
    ASSERT_EQ(NodeStatus::IDLE, parallel_1.status());
    ASSERT_EQ(NodeStatus::IDLE, action_3.status());
    ASSERT_EQ(NodeStatus::IDLE, parallel_2.status());
    ASSERT_EQ(NodeStatus::SUCCESS, state);
}
