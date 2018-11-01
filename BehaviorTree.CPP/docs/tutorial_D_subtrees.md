

One of the main advantages of Behavior Trees is that they are __intrinsically
hierarchical__.

You might have noticed that it is always possible to raise the level of
abstraction looking one node up in the hierarchy of the tree.

In the [Introduction](BT_basics.md) we presented this tree:

![FallbackNodes](images/FallbackBasic.png)  

The Sequence called "Unlock" can be seen as an entire subtree; from the point
of view of its parent, that subtree can have an arbitrary
level of complexity.

__BehaviorTree.CPP__ provides a way to create reusable and composable Subtrees
that can be included as nodes of a larger and more complex tree.

## Example: subtrees in XML

To define and insert a Subtree you __don't need to modify your
 c++ code__, nor your existing TreeNodes implementations.

Multiple BehaviorTrees can be created and composed in the XML itself.


``` XML hl_lines="21"
<root main_tree_to_execute = "MainTree">
	<!--------------------------------------->
    <BehaviorTree ID="DoorClosed">
        <Sequence name="door_closed_sequence">
            <Inverter>
                <IsDoorOpen/>
            </Inverter>
            <RetryUntilSuccesful num_attempts="4">
                <OpenDoor/>
            </RetryUntilSuccesful>
            <PassThroughDoor/>
        </Sequence>
    </BehaviorTree>
    <!--------------------------------------->
    <BehaviorTree ID="MainTree">
        <Fallback name="root_Fallback">
            <Sequence name="door_open_sequence">
                <IsDoorOpen/>
                <PassThroughDoor/>
            </Sequence>
            <SubTree ID="DoorClosed"/>
            <PassThroughWindow/>
        </Fallback>
    </BehaviorTree>
    <!---------------------------------------> 
</root>
```

The corresponding graphical representation is:

![CrossDoorSubtree](images/CrossDoorSubtree.png) 

