#include "Blackboard/blackboard_local.h"
#include "dummy_nodes.h"

using namespace BT;

/**
 * In this first tutorial we demonstrate how to:
 *
 * - Create ActionNodes either from a single function/method or using inheritance
 * - Create a Sequence of Actions.
 * - Build a Tree programmatically.
 */

int main()
{
    using namespace DummyNodes;
    GripperInterface gripper;

    // sequence_root will be the root of our tree
    BT::SequenceNode sequence_root("sequence");

    // Function pointers can be wrapped inside ActionNodeBase
    // using the SimpleActionNode
    SimpleActionNode say_hello("action_hello", std::bind(SayHello));

    // SimpleActionNode works also with class methods, using std::bind
    SimpleActionNode open_gripper("open_gripper", std::bind(&GripperInterface::open, &gripper));
    SimpleActionNode close_gripper("close_gripper", std::bind(&GripperInterface::close, &gripper));

    // To be able to use ALL the functionalities of a TreeNode,
    //  your should create a class that inherits from either:
    // - ConditionNode  (synchronous execution)
    // - ActionNodeBase (synchronous execution)
    // - ActionNode     (asynchronous execution in a separate thread).
    ApproachObject approach_object("approach_object");

    // Add children to the sequence.
    // they will be executed in the same order they are added.
    sequence_root.addChild(&say_hello);
    sequence_root.addChild(&open_gripper);
    sequence_root.addChild(&approach_object);
    sequence_root.addChild(&close_gripper);

    // The tick is propagated to all the children.
    // until one of them returns FAILURE or RUNNING.
    // In this case all of them return SUCCESS immediately
    sequence_root.executeTick();

    // needed when you use ActionNodes with their own thread
    haltAllActions(&sequence_root);

    return 0;
}
