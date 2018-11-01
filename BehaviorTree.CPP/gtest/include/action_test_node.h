#ifndef ACTIONTEST_H
#define ACTIONTEST_H

#include "behavior_tree_core/action_node.h"

namespace BT
{
class SyncActionTest : public ActionNodeBase
{
  public:
    SyncActionTest(const std::string& name);

    BT::NodeStatus tick() override;

    virtual void halt() override
    {
    }

    void setBoolean(bool boolean_value);

    int tickCount() const
    {
        return tick_count_;
    }

  private:
    bool boolean_value_;
    int tick_count_;
};

class AsyncActionTest : public ActionNode
{
  public:
    AsyncActionTest(const std::string& name);

    ~AsyncActionTest();

    // The method that is going to be executed by the thread
    BT::NodeStatus tick() override;

    void setTime(int time);

    // The method used to interrupt the execution of the node
    virtual void halt() override;

    void setBoolean(bool boolean_value);

    int tickCount() const
    {
        return tick_count_;
    }

  private:
    int time_;
    bool boolean_value_;
    int tick_count_;
    std::atomic_bool stop_loop_;
};
}

#endif
