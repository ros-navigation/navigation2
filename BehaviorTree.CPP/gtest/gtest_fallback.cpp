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

struct SimpleFallbackTest : testing::Test
{
    BT::FallbackNode root;
    BT::AsyncActionTest action;
    BT::ConditionTestNode condition;

    SimpleFallbackTest() : root("root_fallback"), action("action"), condition("condition")
    {
        root.addChild(&condition);
        root.addChild(&action);
    }
    ~SimpleFallbackTest()
    {
        haltAllActions(&root);
    }
};

struct ComplexFallbackTest : testing::Test
{
    BT::FallbackNode root;
    BT::AsyncActionTest action_1;
    BT::ConditionTestNode condition_1;
    BT::ConditionTestNode condition_2;

    BT::FallbackNode fal_conditions;

    ComplexFallbackTest()
      : root("root_fallback")
      , action_1("action_1")
      , condition_1("condition_1")
      , condition_2("condition_2")
      , fal_conditions("fallback_conditions")
    {
        root.addChild(&fal_conditions);
        {
            fal_conditions.addChild(&condition_1);
            fal_conditions.addChild(&condition_2);
        }
        root.addChild(&action_1);
    }
    ~ComplexFallbackTest()
    {
        haltAllActions(&root);
    }
};

struct SimpleFallbackWithMemoryTest : testing::Test
{
    BT::FallbackStarNode root;
    BT::AsyncActionTest action;
    BT::ConditionTestNode condition;

    SimpleFallbackWithMemoryTest() : root("root_sequence"), action("action"), condition("condition")
    {
        root.addChild(&condition);
        root.addChild(&action);
    }
    ~SimpleFallbackWithMemoryTest()
    {
        haltAllActions(&root);
    }
};

struct ComplexFallbackWithMemoryTest : testing::Test
{
    BT::FallbackStarNode root;

    BT::AsyncActionTest action_1;
    BT::AsyncActionTest action_2;

    BT::ConditionTestNode condition_1;
    BT::ConditionTestNode condition_2;

    BT::FallbackStarNode fal_conditions;
    BT::FallbackStarNode fal_actions;

    ComplexFallbackWithMemoryTest()
      : root("root_fallback")
      , action_1("action_1")
      , action_2("action_2")
      , condition_1("condition_1")
      , condition_2("condition_2")
      , fal_conditions("fallback_conditions")
      , fal_actions("fallback_actions")
    {
        root.addChild(&fal_conditions);
        {
            fal_conditions.addChild(&condition_1);
            fal_conditions.addChild(&condition_2);
        }
        root.addChild(&fal_actions);
        {
            fal_actions.addChild(&action_1);
            fal_actions.addChild(&action_2);
        }
    }
    ~ComplexFallbackWithMemoryTest()
    {
        haltAllActions(&root);
    }
};

/****************TESTS START HERE***************************/

TEST_F(SimpleFallbackTest, ConditionTrue)
{
    std::cout << "Ticking the root node !" << std::endl << std::endl;
    // Ticking the root node
    condition.setBoolean(true);
    BT::NodeStatus state = root.executeTick();

    ASSERT_EQ(NodeStatus::SUCCESS, state);
    ASSERT_EQ(NodeStatus::IDLE, condition.status());
    ASSERT_EQ(NodeStatus::IDLE, action.status());
}

TEST_F(SimpleFallbackTest, ConditionToFalse)
{
    condition.setBoolean(false);

    BT::NodeStatus state = root.executeTick();
    condition.setBoolean(true);

    state = root.executeTick();

    ASSERT_EQ(NodeStatus::SUCCESS, state);
    ASSERT_EQ(NodeStatus::IDLE, condition.status());
    ASSERT_EQ(NodeStatus::IDLE, action.status());
}

TEST_F(ComplexFallbackTest, Condition1ToTrue)
{
    condition_1.setBoolean(false);
    condition_2.setBoolean(false);

    BT::NodeStatus state = root.executeTick();

    ASSERT_EQ(NodeStatus::RUNNING, state);
    ASSERT_EQ(NodeStatus::FAILURE, fal_conditions.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::RUNNING, action_1.status());

    condition_1.setBoolean(true);

    state = root.executeTick();

    ASSERT_EQ(NodeStatus::SUCCESS, state);
    ASSERT_EQ(NodeStatus::IDLE, fal_conditions.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::IDLE, action_1.status());
}

TEST_F(ComplexFallbackTest, Condition2ToTrue)
{
    condition_1.setBoolean(false);
    condition_2.setBoolean(false);

    BT::NodeStatus state = root.executeTick();

    ASSERT_EQ(NodeStatus::RUNNING, state);
    ASSERT_EQ(NodeStatus::FAILURE, fal_conditions.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::RUNNING, action_1.status());

    condition_2.setBoolean(true);

    state = root.executeTick();

    ASSERT_EQ(NodeStatus::SUCCESS, state);
    ASSERT_EQ(NodeStatus::IDLE, fal_conditions.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::IDLE, action_1.status());
}

TEST_F(SimpleFallbackWithMemoryTest, ConditionFalse)
{
    condition.setBoolean(false);
    BT::NodeStatus state = root.executeTick();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ASSERT_EQ(NodeStatus::RUNNING, state);
    ASSERT_EQ(NodeStatus::FAILURE, condition.status());
    ASSERT_EQ(NodeStatus::RUNNING, action.status());
}

TEST_F(SimpleFallbackWithMemoryTest, ConditionTurnToTrue)
{
    condition.setBoolean(false);
    BT::NodeStatus state = root.executeTick();

    ASSERT_EQ(NodeStatus::RUNNING, state);
    ASSERT_EQ(NodeStatus::FAILURE, condition.status());
    ASSERT_EQ(NodeStatus::RUNNING, action.status());

    condition.setBoolean(true);
    state = root.executeTick();

    ASSERT_EQ(NodeStatus::RUNNING, state);
    ASSERT_EQ(NodeStatus::FAILURE, condition.status());
    ASSERT_EQ(NodeStatus::RUNNING, action.status());
}

TEST_F(ComplexFallbackWithMemoryTest, ConditionsTrue)
{
    BT::NodeStatus state = root.executeTick();

    ASSERT_EQ(NodeStatus::SUCCESS, state);
    ASSERT_EQ(NodeStatus::IDLE, fal_conditions.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::IDLE, fal_actions.status());
    ASSERT_EQ(NodeStatus::IDLE, action_1.status());
    ASSERT_EQ(NodeStatus::IDLE, action_2.status());
}

TEST_F(ComplexFallbackWithMemoryTest, Condition1False)
{
    condition_1.setBoolean(false);
    BT::NodeStatus state = root.executeTick();

    ASSERT_EQ(NodeStatus::SUCCESS, state);
    ASSERT_EQ(NodeStatus::IDLE, fal_conditions.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::IDLE, fal_actions.status());
    ASSERT_EQ(NodeStatus::IDLE, action_1.status());
    ASSERT_EQ(NodeStatus::IDLE, action_2.status());
}

TEST_F(ComplexFallbackWithMemoryTest, ConditionsFalse)
{
    condition_1.setBoolean(false);
    condition_2.setBoolean(false);
    BT::NodeStatus state = root.executeTick();

    ASSERT_EQ(NodeStatus::RUNNING, state);
    ASSERT_EQ(NodeStatus::FAILURE, fal_conditions.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::RUNNING, fal_actions.status());
    ASSERT_EQ(NodeStatus::RUNNING, action_1.status());
    ASSERT_EQ(NodeStatus::IDLE, action_2.status());
}

TEST_F(ComplexFallbackWithMemoryTest, Conditions1ToTrue)
{
    condition_1.setBoolean(false);
    condition_2.setBoolean(false);
    BT::NodeStatus state = root.executeTick();

    condition_1.setBoolean(true);
    state = root.executeTick();

    ASSERT_EQ(NodeStatus::RUNNING, state);
    ASSERT_EQ(NodeStatus::FAILURE, fal_conditions.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::RUNNING, fal_actions.status());
    ASSERT_EQ(NodeStatus::RUNNING, action_1.status());
    ASSERT_EQ(NodeStatus::IDLE, action_2.status());
}

TEST_F(ComplexFallbackWithMemoryTest, Conditions2ToTrue)
{
    condition_1.setBoolean(false);
    condition_2.setBoolean(false);
    BT::NodeStatus state = root.executeTick();

    condition_2.setBoolean(true);
    state = root.executeTick();

    ASSERT_EQ(NodeStatus::RUNNING, state);
    ASSERT_EQ(NodeStatus::FAILURE, fal_conditions.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::RUNNING, fal_actions.status());
    ASSERT_EQ(NodeStatus::RUNNING, action_1.status());
    ASSERT_EQ(NodeStatus::IDLE, action_2.status());
}

TEST_F(ComplexFallbackWithMemoryTest, Action1Failed)
{
    action_1.setBoolean(false);
    condition_1.setBoolean(false);
    condition_2.setBoolean(false);

    BT::NodeStatus state = root.executeTick();

    state = root.executeTick();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    state = root.executeTick();

    ASSERT_EQ(NodeStatus::RUNNING, state);
    ASSERT_EQ(NodeStatus::FAILURE, fal_conditions.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_1.status());
    ASSERT_EQ(NodeStatus::IDLE, condition_2.status());
    ASSERT_EQ(NodeStatus::RUNNING, fal_actions.status());
    ASSERT_EQ(NodeStatus::FAILURE, action_1.status());
    ASSERT_EQ(NodeStatus::RUNNING, action_2.status());
}
