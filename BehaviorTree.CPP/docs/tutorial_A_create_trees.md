# How to create a BehaviorTree

You have mainly two ways to create Behavior Trees.

- __Statically__, at compilation time.
- __Dynamically__, at run-time, i.e. parsing a file.

You are __strongly encourage to use the latter approach__, but we will describe
the former for the sake of completeness.

## How to create your own ActionNodes

You can find the source code here: **sample_nodes/dummy_nodes.h**.

The default (and recommended) way to create a TreeNode is by inheritance.

``` c++
// Example of custom ActionNodeBase (synchronous Action)
class ApproachObject: public BT::ActionNodeBase
{
public:
    ApproachObject(const std::string& name):
        BT::ActionNodeBase(name) {}

    // You must override this virtual function
    BT::NodeStatus tick() override
    {
		std::cout << "ApproachObject: " << this->name() << std::endl;
		return BT::NodeStatus::SUCCESS;
	}

    // You must override this virtual function
    virtual void halt() override 
    {
		// Do nothing. This is used by asynchronous nodes only.
    }
};
``` 

As you can see:

- Any instance of a TreeNode has a name. This identifier is meant to be user-readable and it 
 doesn't need to be unique.
 
- The method __tick()__ is the place where the actual Action takes place.
It must return a NodeStatus, i.e. RUNNING, SUCCESS or FAILURE. 

- The method __halt()__ is used to stop an __asynchronous Action__. ApproachObject
doesn't need it.
 
Alternatively, we can use __dependecy injection__ to create a TreeNode given 
a function pointer. 

The only requirement of the functor is to have either one of these signatures:

``` c++
    BT::NodeStatus myFunction()
    BT::NodeStatus myFunction(BT::TreeNode& self) 
```

For example:


``` c++
using namespace BT;

NodeStatus SayHello() {
    std::cout << "Robot says Hello" << std::endl;
    return NodeStatus::SUCCESS;
}

class GripperInterface
{
public:
    GripperInterface(): _open(true) {}
    
	NodeStatus open() {
		_open = true;
		std::cout << "GripperInterface::open" << std::endl;
		return NodeStatus::SUCCESS;
	}

	NodeStatus close() {
		std::cout << "GripperInterface::close" << std::endl;
		_open = false;
		return NodeStatus::SUCCESS;
	}

private:
    bool _open;
};

``` 

We can build a `SimpleActionNode` from any of these functors:

- SayHello()
- GripperInterface::open()
- GripperInterface::close()

## A static Tree

Let's create instances of our TreeNodes and compose them into a tree.

- `BT::SequenceNode` is a built-in ControlNode provided by the library.
- `BT::SimpleActionNode` is a synchronous ActionNode created passing a functor.
- `DummyNodes::ApproachObject` is our user-defined ActionNode.

``` c++ 
#include "dummy_nodes.h"

int main()
{
	using namespace BT;
    using namespace DummyNodes;
    
    GripperInterface gi;

    SequenceNode sequence_root("sequence");
    SimpleActionNode say_hello("action_hello", std::bind(SayHello) );
    SimpleActionNode open_gripper("open_gripper",   
                                  std::bind( &GripperInterface::open,  &gi) );
    SimpleActionNode close_gripper("close_gripper", 
                                   std::bind( &GripperInterface::close, &gi) );
    ApproachObject approach_object("approach_object");

    // Add children to the sequence. 
    // They will be executed in the same order they are added.
    sequence_root.addChild(&say_hello);
    sequence_root.addChild(&open_gripper);
    sequence_root.addChild(&approach_object);
    sequence_root.addChild(&close_gripper);

    sequence_root.executeTick();
    return 0;
}

/* Expected output:

    Robot says: "Hello!!!"
    GripperInterface::open
    ApproachObject: approach_object
    GripperInterface::close
*/

``` 

## A dynamically created Tree

Give the following XML stored in the file __my_tree.xml__

``` XML
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <SayHello       name="action_hello"/>
            <OpenGripper    name="open_gripper"/>
            <ApproachObject name="approach_object"/>
            <CloseGripper   name="close_gripper"/>
        </Sequence>
     </BehaviorTree>
 </root>
```

Note that the following syntax is also valid:

``` XML
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <Action ID="SayHello"       name="action_hello"/>
            <Action ID="OpenGripper"    name="open_gripper"/>
            <Action ID="ApproachObject" name="approach_object"/>
            <Action ID="CloseGripper"   name="close_gripper"/>
        </Sequence>
     </BehaviorTree>
 </root>
```

We must first register our custom TreeNodes into the `BehaviorTreeFactory`
 and then load the XML from file or text.

The identifier used in the XML must coincide with those used to register
the TreeNodes.

The attribute "name" represents the name of the instance and it is optional.


``` c++
#include "behavior_tree_core/xml_parsing.h"
#include "Blackboard/blackboard_local.h"
#include "dummy_nodes.h"

int main()
{
	using namespace BT;
    using namespace DummyNodes;

    GripperInterface gi;    
    BehaviorTreeFactory factory;

    factory.registerSimpleAction("SayHello", std::bind(SayHello) );
    factory.registerSimpleAction("OpenGripper", 
                                 std::bind( &GripperInterface::open, &gi));
    factory.registerSimpleAction("CloseGripper", 
                                  std::bind( &GripperInterface::close, &gi));

    factory.registerNodeType<ApproachObject>("ApproachObject");

    // IMPORTANT: when the object "tree" goes out of scope,
    // all the TreeNodes instances are destroyed
    auto tree = buildTreeFromFile(factory, "./my_tree.xml");

    tree.root_node->executeTick();
    return 0;
}

/* Expected output:

    Robot says: "Hello!!!"
    GripperInterface::open
    ApproachObject: approach_object
    GripperInterface::close
*/

``` 



