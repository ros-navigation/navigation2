# Getting started

__BehaviorTree.CPP__ is a C++ library that can be easily integrated into
your favourite distributed middleware, such as __ROS__ or __SmartSoft__.

You can also statically link it into your application (for example a game).

There are some main concepts  that you need to understand first.

## Nodes vs Trees

The user must create his/her own ActionNodes and ConditionNodes (LeafNodes) and this 
library helps you to compose them easily into trees. 

Think about the LeafNodes as the building blocks that you need to compose
a complex system.

By definition, your custom Nodes are (or should be) highly reusable.
Therefore, some wrapping interfaces might be needed at the beginning to adapt your
legacy code.


## The tick() callbacks

Any TreeNode can be seen as a mechanism to invoke a __callback__, i.e. to 
__run a piece of code__. What this callback does is up to you.

In most of the __following examples__, our Actions just
print messages on the screen of sleep for a certain amount of time to simulate
a long calculation.

## Inheritance vs dependency injection.

To create a custom TreeNode, you should inherit from the proper class.

For instance, to create your own synchronous Action, you should inherit from the 
class __ActionNodeBase__. 

Alternatively, we provided a mechanism to create a TreeNode passing a 
__function pointer__ to a wrapper (dependency injection).

This approach reduces the amount of boilerplate in your code but has also 
some limitations; the most important one is that TreeNodes created using 
function pointers can not support [NodeParameters](NodeParameters.md).

## NodeParameters

NodeParameters are conceptually similar to the arguments of a function.

They are passed statically when the tree is instantiated.

They are expressed as a list of __key/value__ pairs, where both the the
key and the value are strings.

This is not surprising, since NodeParameters are usually parsed from file.

The library provides some methods and utility functions to correctly convert
values from string to the desired C++ type.  

