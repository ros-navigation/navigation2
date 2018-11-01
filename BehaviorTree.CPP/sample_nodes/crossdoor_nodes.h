#include "behavior_tree_core/bt_factory.h"

using namespace BT;

namespace CrossDoor
{
inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

BT::NodeStatus IsDoorOpen(TreeNode& self);

BT::NodeStatus IsDoorLocked(TreeNode& self);

BT::NodeStatus UnlockDoor(TreeNode& self);

BT::NodeStatus PassThroughDoor(TreeNode& self);

BT::NodeStatus PassThroughWindow(TreeNode& self);

BT::NodeStatus OpenDoor(TreeNode& self);

BT::NodeStatus CloseDoor(TreeNode& self);

void RegisterNodes(BT::BehaviorTreeFactory& factory);
}
