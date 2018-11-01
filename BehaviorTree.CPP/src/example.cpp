#include <iostream>
#include <behavior_tree.h>

class MyCondition : public BT::ConditionNode
{
  public:
    MyCondition(const std::string& name);
    ~MyCondition();
    BT::ReturnStatus Tick();
};

MyCondition::MyCondition(const std::string& name) : BT::ConditionNode::ConditionNode(name)
{
}

BT::ReturnStatus MyCondition::Tick()
{
    std::cout << "The Condition is true" << std::endl;

    return NodeStatus::SUCCESS;
}

class MyAction : public BT::ActionNode
{
  public:
    MyAction(const std::string& name);
    ~MyAction();
    BT::ReturnStatus Tick();
    void Halt();
};

MyAction::MyAction(const std::string& name) : ActionNode::ActionNode(name)
{
}

BT::ReturnStatus MyAction::Tick()
{
    std::cout << "The Action is doing some operations" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (is_halted())
    {
        return NodeStatus::IDLE;
    }

    std::cout << "The Action is doing some others operations" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (is_halted())
    {
        return NodeStatus::IDLE;
    }

    std::cout << "The Action is doing more operations" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (is_halted())
    {
        return NodeStatus::IDLE;
    }

    std::cout << "The Action has succeeded" << std::endl;
    return NodeStatus::SUCCESS;
}

void MyAction::Halt()
{
}

int main(int argc, char* argv[])
{
    BT::SequenceNode* seq = new BT::SequenceNode("Sequence");
    MyCondition* my_con_1 = new MyCondition("Condition");
    MyAction* my_act_1 = new MyAction("Action");
    int tick_time_milliseconds = 1000;

    seq->AddChild(my_con_1);
    seq->AddChild(my_act_1);

    Execute(seq, tick_time_milliseconds);

    return 0;
}
