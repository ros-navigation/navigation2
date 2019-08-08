# nav2_behavior_tree

The nav2_behavior_tree module provides a C++ template class for integrating ROS2 actions into Behavior Trees, navigation-specific behavior tree nodes, and a generic BehaviorTreeEngine class that simplifies the integration of BT processing into ROS2 nodes. This module is used by the nav2_bt_navigator to implement a ROS2 node that executes navigation Behavior Trees. The nav2_behavior_tree module uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) for the core Behavior Tree processing. 

## The bt_action_node Template

The [bt_action_node template](include/nav2_behavior_tree/bt_action_node.hpp) allows one to easily integrate a ROS2 action into a BehaviorTree. To do so, one derives from the BTActionNode template, providing the action message type. For example,

```C++
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

class FollowPathAction : public BtActionNode<nav2_msgs::action::FollowPath>
{
    ...
};
```
The resulting nodes must be registered in the Behavior Tree engine in order to be used in Behavior Trees executed by this engine.

```C++
BehaviorTreeEngine::BehaviorTreeEngine()
{
    ...

  factory_.registerNodeType<nav2_behavior_tree::FollowPathAction>("FollowPath");

    ...
}
```

Once a new node is registered in this way, it is now available to the BehaviorTreeEngine and can be used in Behavior Trees. For example,

```XML
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <ComputePathToPose goal="${goal}"/>
      <FollowPath path="${path}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

## The Behavior Tree Engine

The BT Navigator package has two sample XML-based descriptions of BTs.  These trees are [navigate_w_replanning.xml](behavior_trees/navigate_w_replanning.xml) and [navigate_w_replanning_and_recovery.xml](behavior_trees/navigate_w_replanning_and_recovery.xml).  The user may use any of these sample trees or develop a more complex tree which could better suit the user's needs.

## Navigation-Specific Behavior Tree Nodes

A Behavior Tree consists of control flow nodes, such as fallback, sequence, parallel, and decorator, as well as two execution nodes: condition and action nodes. Execution nodes are the leaf nodes of the tree. When a leaf node is ticked, the node does some work and it returns either SUCCESS, FAILURE or RUNNING.  The current Navigation2 software implements a few custom nodes, including Conditions and Actions. The user can also define and register additional node types that can then be used in BTs and the corresponding XML descriptions.

#### Decorator Nodes
* **RateController**: A custom control flow node, which throttles down the tick rate.  This custom node has only one child and its tick rate is defined with a pre-defined frequency that the user can set.  This node returns RUNNING when it is not ticking its child. Currently, in the navigation, the `RateController` is used to tick the  `ComputePathToPose` and `GoalReached` node at 1 Hz.

#### Condition Nodes
* **GoalReached**: Checks the distance to the goal, if the distance to goal is less than the pre-defined threshold, the tree returns SUCCESS, which in that case the `ComputePathToPose` action node will not get ticked. 

#### Action Nodes
* **ComputePathToPose**: When this node is ticked, the goal will be placed on the blackboard which will be shared to the Behavior tree.  The bt action node would then utilizes the action server to send a request to the global planner to recompute the global path.  Once the global path is recomputed, the result will be sent back via action server and then the updated path will be placed on the blackboard.

#### Recovery Node
In this section, the recovery node is being introduced to the navigation package.

Recovery node is a control flow type node with two children.  It returns success if and only if the first child returns success. The second child will be executed only if the first child returns failure.  The second child is responsible for recovery actions such as re-initializing system or other recovery behaviors. If the recovery behaviors are succeeded, then the first child will be executed again.  The user can specify how many times the recovery actions should be taken before returning failure. The figure below depicts a simple recovery node.

<img src="./doc/recovery_node.png" title="" width="40%" align="middle">
<br/>

## Example Behavior Tree

The graphical version of this Behavior Tree:

<img src="./doc/simple_parallel.png" title="" width="65%" align="middle">
<br/>

<img src="./doc/proposed_recovery.png" title="" width="95%" align="middle">
<br/>

The navigate with replanning BT first ticks the `RateController` node which specifies how frequently the `GoalReached` and `ComputePathToPose` should be invoked. Then the `GoalReached` nodes check the distance to the goal to determine if the `ComputePathToPose` should be ticked or not. The `ComputePathToPose` gets the incoming goal pose from the blackboard, computes the path and puts the result back on the blackboard, where `FollowPath` picks it up. Each time a new path is computed, the blackboard gets updated and then `FollowPath` picks up the new goal.

## Future Work
* **Schema definition and XML document validation** - Currently, there is no dynamic validation of incoming XML. The Behavior-Tree.CPP library is using tinyxml2, which doesn't have a validator. Instead, we can create a schema for the Mission Planning-level XML and use build-time validation of the XML input to ensure that it is well-formed and valid.
* **Port to BT 3.0**
* ** Use plug-ins to simplify integration of user BT nodes**
