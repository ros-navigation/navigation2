#include "crossdoor_nodes.h"

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
    CrossDoor::RegisterNodes(factory);
}

NodeStatus CrossDoor::IsDoorOpen(TreeNode& self)
{
    SleepMS(500);
    bool door_open = self.blackboard()->get<bool>("door_open");

    return door_open ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus CrossDoor::IsDoorLocked(TreeNode& self)
{
    SleepMS(500);
    bool door_locked = self.blackboard()->get<bool>("door_locked");

    return door_locked ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus CrossDoor::UnlockDoor(TreeNode& self)
{
    SleepMS(2000);
    self.blackboard()->set("door_locked", false);

    return NodeStatus::SUCCESS;
}

NodeStatus CrossDoor::PassThroughDoor(TreeNode& self)
{
    SleepMS(1000);
    bool door_open = self.blackboard()->get<bool>("door_open");

    return door_open ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus CrossDoor::PassThroughWindow(TreeNode& self)
{
    SleepMS(1000);
    return NodeStatus::SUCCESS;
}

NodeStatus CrossDoor::OpenDoor(TreeNode& self)
{
    SleepMS(2000);
    bool door_locked = self.blackboard()->get<bool>("door_locked");

    if (door_locked)
    {
        return NodeStatus::FAILURE;
    }

    self.blackboard()->set("door_open", true);
    return NodeStatus::SUCCESS;
}

NodeStatus CrossDoor::CloseDoor(TreeNode& self)
{
    bool door_open = self.blackboard()->get<bool>("door_open");

    if (door_open)
    {
        SleepMS(1500);
        self.blackboard()->set("door_open", false);
    }
    return NodeStatus::SUCCESS;
}

void CrossDoor::RegisterNodes(BehaviorTreeFactory& factory)
{
    factory.registerSimpleCondition("IsDoorOpen", IsDoorOpen);
    factory.registerSimpleAction("PassThroughDoor", PassThroughDoor);
    factory.registerSimpleAction("PassThroughWindow", PassThroughWindow);
    factory.registerSimpleAction("OpenDoor", OpenDoor);
    factory.registerSimpleAction("CloseDoor", CloseDoor);
    factory.registerSimpleCondition("IsDoorLocked", IsDoorLocked);
    factory.registerSimpleAction("UnlockDoor", UnlockDoor);
}
