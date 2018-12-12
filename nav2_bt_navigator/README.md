# BT Navigator

The BT Navigator module implements the [NavigateToPose task interface](../nav2_tasks/include/nav2_tasks/navigate_to_pose_task.hpp). It is a Behavior Tree-based implementation of navigation that is intended to allow for flexibility in the navigation task and provide a way to easily specify complex robot behaviors, including recovery.

## Overview

The BT Navigator receives a goal pose and navigates the robot to the specified destination. To do so, the module reads an XML description of the Behavior Tree from a file, as specified by a Node parameter, and passes that to a generic [BehaviorTreeEngine class](../nav2_tasks/include/nav2_tasks/behavior_tree_engine.hpp) which uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) to dynamically create and execute the BT. 

## Specifying an input XML file 

The BtNavigator node has a parameter, *bt_xml_filename*, that can be specified using a ROS2 parameters YAML file, like this:

```
BtNavigator:
  ros__parameters:
    bt_xml_filename: <path-to-xml-file>
```

Using the XML filename as a parameter makes it easy to change or extend the logic used for navigation. Once can simply update the XML description for the BT and the BtNavigator task server will use the new description.

## Example Behavior Trees

The BT Navigator package has a few sample XML-based descriptions of BTs. 

### Simple sequential invocation of planning and control

[Simple_sequential.xml](behavior_trees/simple_sequential.xml) implements a basic navigation by first computing the path and then following the path. 

```XML
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SequenceStar name="root">
      <ComputePathToPose goal="${goal}" path="${path}"/>
      <FollowPath path="${path}"/>
    </SequenceStar>
  </BehaviorTree>
</root>
```

The graphical version of this Behavior Tree:

<img src="./doc/simple.png" title="Simple Navigation Behavior Tree">

**ComputePathToPose** gets the incoming goal pose from the blackboard, computes the path and puts the result back on the blackboard, where **FollowPath** picks it up.

### Parallel planning and control

An alternative approach is to run planning and control in parallel. [Parallel.xml](behavior_trees/parallel.xml) implements one possible BT for doing this:

```XML
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SequenceStar name="root">
      <ComputePathToPose goal="${goal}" path="${path}"/>
      <ParallelNode threshold="1">
        <FollowPath path="${path}"/>
        <Sequence>
          <RateController hz="1.0">
            <ComputePathToPose goal="${goal}" path="${path}"/>
          </RateController>
          <UpdatePath/>
        </Sequence>
      </ParallelNode>
    </SequenceStar>
  </BehaviorTree>
</root>
```

The graphical version of this Behavior Tree:

<img src="./doc/parallel.png" title="Parallel version of the Navigation Behavior Tree">

In this case, the BT first calls **ComputePathToPose** to generate an initial path. It then runs **FollowPath** and a rate-controlled **ComputePathToPose** in parallel. The **RateController** node specifies how frequently the **ComputePathToPose** task should be invoked. Each time a new path is computed, it is sent to the local planner/controller using **UpdatePath**.

### Recovery 

There are versions of the above BTs, [simple_sequential_w_recovery.xml](behavior_trees/simple_sequential_w_recovery.xml) and [parallel_w_recovery.xml](behavior_trees/parallel_w_recovery.xml) that add recovery sub-trees to the navigation task. 

For example, in the simple_sequential version, there is node, **IsStuck** that checks whether the robot is no longer making progress toward its goal pose. If this condition is detected, a few maneuvers - **Stop**, **BackUp**, and **Spin** - are executed to attempt to free up the robot. 

```XML
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <FallbackStar name="check_motion">
        <Inverter name="is_not_stuck">
          <IsStuck/>
        </Inverter>
        <SequenceStar name="stuck_recovery">
          <Stop/>
          <BackUp/>
          <Spin/>
        </SequenceStar>
      </FallbackStar>
      <SequenceStar name="navigate">
        <ComputePathToPose endpoints="${endpoints}" path="${path}"/>
        <FollowPath path="${path}"/>
      </SequenceStar>
    </Sequence>
  </BehaviorTree>
</root>
```

## Creating custom Behavior Tree nodes

A Behavior Tree consists of various kinds of nodes: control flow nodes, such as fallback, sequence, parallel, and decorator, as well as condition and action nodes. The current Navigation2 software implements a few custom nodes, including Conditions and Actions. The user can also define and register additional node types that can then be used in BTs and the corresponding XML descriptions. See the code for examples. 

## Open Issues

* **Schema definition and XML document validation** - Currently, there is no dynamic validation of incoming XML. The Behavior-Tree.CPP library is using tinyxml2, which doesn't have a validator. Instead, we can create a schema for the Mission Planning-level XML and use build-time validation of the XML input to ensure that it is well-formed and valid.
