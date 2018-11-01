#include "Blackboard/blackboard_local.h"
#include "behavior_tree_core/behavior_tree.h"
#include "behavior_tree_core/bt_factory.h"

using namespace BT;

NodeStatus SayHello()
{
    printf("hello\n");
    return NodeStatus::SUCCESS;
}

class ActionTestNode : public ActionNode
{
  public:
    ActionTestNode(const std::string& name) : ActionNode(name)
    {
    }

    NodeStatus tick() override
    {
        time_ = 5;
        stop_loop_ = false;
        int i = 0;
        while (!stop_loop_ && i++ < time_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return NodeStatus::SUCCESS;
    }

    virtual void halt() override
    {
        stop_loop_ = true;
        setStatus(NodeStatus::IDLE);
    }

  private:
    int time_;
    std::atomic_bool stop_loop_;
};

int main()
{
    BT::SequenceNode root("root");
    BT::SimpleActionNode action1("say_hello", std::bind(SayHello));
    ActionTestNode action2("async_action");

    root.addChild(&action1);
    root.addChild(&action2);

    int count = 0;

    NodeStatus status = NodeStatus::RUNNING;

    while (status == NodeStatus::RUNNING)
    {
        status = root.executeTick();

        std::cout << count++ << " : " << root.status() << " / " << action1.status() << " / "
                  << action2.status() << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    haltAllActions(&root);

    return 0;
}
// Output
/*

hello
0 : RUNNING / SUCCESS / RUNNING
hello
1 : RUNNING / SUCCESS / RUNNING
hello
2 : RUNNING / SUCCESS / RUNNING
hello
3 : RUNNING / SUCCESS / RUNNING
hello
4 : RUNNING / SUCCESS / RUNNING
hello
5 : SUCCESS / IDLE / IDLE

*/
