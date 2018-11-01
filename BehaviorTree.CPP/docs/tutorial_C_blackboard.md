# Blackboards

The blackboard is a a __key/value__ storage shared by all the Nodes
of a tree.

The __key__ is a string whilst the __value__ is a type-erased container (called `SafeAny::Any`) 
that allows the user to store any C++ object and to cast it back to its original form.

Contrariwise to `boost::any` and `std::any`, `SafeAny::Any` will also try to 
avoid common overflow and underflow errors.

In fact, you can't cast a negative number into an `unsigned integer`,
nor a very large number that exceeds 2^8 into a `char`. 

If the __value__ is stored as a string, the blackboard will use `convertFromString<T>()`
to cast it to the type T (see [previous example](tutorial_B_node_parameters.md));

The user can create his/her own Blackboards backend; it is possible, for instance,
to create a persistent blackboard using a database.

## Assign a blackboard to a tree

Let's start with the a `SimpleActionNode` that writes into the blackboard.

``` c++
// Write into the blackboard key: [GoalPose]
// Use this function to create a SimpleActionNode
NodeStatus CalculateGoalPose(TreeNode& self)
{
    const Pose2D mygoal = { 1, 2, 3.14};

    // RECOMMENDED: check if the blackboard is nullptr first
    if( self.blackboard() )
    {
        // store it in the blackboard
        self.blackboard()->set("GoalPose", mygoal);
        return NodeStatus::SUCCESS;
    }
    else{
		// No blackboard passed to this node.
		return NodeStatus::FAILURE;
	}
}
```

Let's consider the following XML tree definition:

``` XML
 <root main_tree_to_execute = "MainTree">
     <BehaviorTree ID="MainTree">
        <SequenceStar>
            <CalculateGoalPose/>
            <MoveBase  goal="${GoalPose}" />
            <SetBlackboard key="OtherGoal" value="-1;3;0.5" />
            <MoveBase  goal="${OtherGoal}" />
        </SequenceStar>
     </BehaviorTree>
 </root>
```

The root SequenceStar will execute four actions:

- `CalculateGoalPose` writes into the blackboard key "GoalPose".
- The syntax `${...}` tells to `MoveBase` to read the goal at run-time from the blackboard;
  they blackboard key is "GoalPose".
- Alternatively, you can write a key/value pair into the blackboard using the built-in action `SetBlackboard`.
- Similar to step 2. Pose2D is retrieved from the key "OtherGoal".  

!!! note
    For your information, __GoalPose__ is stored as a type erased Pose2D.
    
    On the other hand, __OtherGoal__ is stored as a std::string, but is converted 
    to Pose2D by the method `getParam()` using the function `convertFromString<Pose2D>()`.

In the following code sample we can see two equivalent ways to assign a 
Blackboard to a tree.

``` c++ hl_lines="13 14 15 16"
int main()
{
    using namespace BT;

    BehaviorTreeFactory factory;
    factory.registerSimpleAction("CalculateGoalPose", CalculateGoalPose);
    factory.registerNodeType<MoveBaseAction>("MoveBase");

    // create a Blackboard from BlackboardLocal (simple, not persistent, local storage)
    auto blackboard = Blackboard::create<BlackboardLocal>();

    // Important: when the object tree goes out of scope, all the TreeNodes are destroyed
    auto tree = buildTreeFromText(factory, xml_text, blackboard);
    // alternatively:
    //  auto tree = buildTreeFromText(factory, xml_text);
    //  assignBlackboardToEntireTree( tree.root_node, blackboard );

    NodeStatus status = NodeStatus::RUNNING;
    while( status == NodeStatus::RUNNING )
    {
        status = tree.root_node->executeTick();
        SleepMS(1); // optional sleep to avoid "busy loops"
    }
    return 0;
}

/* Expected output

[ MoveBase: STARTED ]. goal: x=1 y=2.0 theta=3.14
[ MoveBase: FINISHED ]
[ MoveBase: STARTED ]. goal: x=-1 y=3.0 theta=0.50
[ MoveBase: FINISHED ]
*/

```
