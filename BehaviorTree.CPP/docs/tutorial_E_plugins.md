# Plugins

In the previous examples the user-defined nodes where included
and linked statically into out C++ projects.

We used the `BehaviorTreeFactory` to registed manualy these custom TreeNodes.

Alternatively, we can load user-defined TreeNodes at run-time using 
pre-compiled __dynamic shared libraries, i.e. plugins__.

# Example

Let's consider the [first tutorial](tutorial_A_create_trees.md).

To create a plugin we must encapsulate the registration of one or multiple TreeNodes 
into a single function like this:

``` c++
// This is a macro that defines a function with a single argument 
// (BehaviorTreeFactory& factory)

BT_REGISTER_NODES(factory)
{
    static GripperInterface gi; // we can't have more than instance
    
    factory.registerSimpleAction("SayHello", std::bind(SayHello) );
    factory.registerSimpleAction("OpenGripper",  
                                 std::bind( &GripperInterface::open, &gi));
    factory.registerSimpleAction("CloseGripper", 
                                 std::bind( &GripperInterface::close, &gi));
    factory.registerNodeType<ApproachObject>("ApproachObject");
    factory.registerNodeType<SaySomething>("SaySomething");
}
```

!!! note
    This function must be placed in __.cpp__ file, not the header file.
    
Here, we assume that BT_REGISTER_NODES and
the definitions of our custom TreeNodes are all defined in the file __dummy_nodes.cpp__.

When you use __cmake__ to compile a plugin, add the argument `SHARED` to
`add_library`.

```cmake
#your CMakeLists.txt
add_library(dummy_nodes  SHARED dummy_nodes.cpp )
``` 

In Linux, the file __libdummy_nodes.so__ will be created.

The [first tutorial](tutorial_A_create_trees.md) becomes, as a result, much simpler:


```c++ hl_lines="3 25"
#include "behavior_tree_core/xml_parsing.h"
#include "Blackboard/blackboard_local.h"
// #include "dummy_nodes.h" YOU DON'T NEED THIS ANYMORE

using namespace BT;

const std::string xml_text = R"(
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
)";

int main()
{
	using namespace BT;
	
    BehaviorTreeFactory factory;
    factory.registerFromPlugin("./libdummy_nodes.so");

    auto tree = buildTreeFromText(factory, xml_text);

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

## Display the manifest of a plugin

BehaviorTree.CPP provides a command line tool called 
__bt_plugin_manifest__.

It shows all user-defind TreeNodes
registered into the plugin and their corresponding NodeParameters.


```
$> ./bt_plugin_manifest ./libdummy_nodes.so 

---------------
ApproachObject [Action]
  NodeParameters: 0
---------------
CloseGripper [Action]
  NodeParameters: 0
---------------
OpenGripper [Action]
  NodeParameters: 0
---------------
SayHello [Action]
  NodeParameters: 0
---------------
SaySomething [Action]
  NodeParameters: 1:
    - [Key]: "message" / [Default]: "" 
```






