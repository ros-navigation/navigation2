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

#include <behavior_tree.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "BehaviorTree");
    try
    {
        int TickPeriod_milliseconds = 1000;

        BT::ActionTestNode* action1 = new BT::ActionTestNode("Action 1");
        // BT::ConditionTestNode* condition1 = new BT::ConditionTestNode("Condition 1");  // commented-out as unused
        BT::SequenceNode* sequence1 = new BT::SequenceNode("seq1");

        BT::ActionTestNode* action2 = new BT::ActionTestNode("Action 2");
        BT::ConditionTestNode* condition2 = new BT::ConditionTestNode("Condition 2");
        BT::SequenceNode* sequence2 = new BT::SequenceNode("seq1");

        BT::ActionTestNode* action3 = new BT::ActionTestNode("Action 3");
        BT::ConditionTestNode* condition3 = new BT::ConditionTestNode("Condition 3");
        BT::SequenceNode* sequence3 = new BT::SequenceNode("seq1");

        // Commented-out as unused variables
        // BT::ActionTestNode* action4 = new BT::ActionTestNode("Action 4");
        // BT::ConditionTestNode* condition4 = new BT::ConditionTestNode("Condition 4");
        // BT:: SequenceNode* sequence4 = new BT::SequenceNode("seq1");

        sequence1->AddChild(condition2);
        sequence1->AddChild(action1);
        sequence1->AddChild(sequence2);
        sequence1->AddChild(action3);
        sequence1->AddChild(sequence2);

        sequence2->AddChild(action2);
        sequence2->AddChild(condition2);

        sequence3->AddChild(condition3);
        sequence3->AddChild(action3);

        Execute(sequence1, TickPeriod_milliseconds);   // from BehaviorTree.cpp
    }
    catch (BT::BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

    return 0;
}
