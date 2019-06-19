# BT Navigator

The BT Navigator (Behavior Tree Navigator) module implements the [NavigateToPose task interface](../nav2_behavior_tree/include/nav2_behavior_tree/navigate_to_pose_task.hpp). It is a [Behavior Tree](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/docs/BT_basics.md)-based implementation of navigation that is intended to allow for flexibility in the navigation task and provide a way to easily specify complex robot behaviors, including [recovery](#recovery).

## Overview

The BT Navigator receives a goal pose and navigates the robot to the specified destination. To do so, the module reads an XML description of the Behavior Tree from a file, as specified by a Node parameter, and passes that to a generic [BehaviorTreeEngine class](../nav2_behavior_tree/include/nav2_behavior_tree/behavior_tree_engine.hpp) which uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) to dynamically create and execute the BT.

## Specifying an input XML file

The BtNavigator node has a parameter, *bt_xml_filename*, that can be specified using a ROS2 parameters YAML file, like this:

```
BtNavigator:
  ros__parameters:
    bt_xml_filename: <path-to-xml-file>
```

Using the XML filename as a parameter makes it easy to change or extend the logic used for navigation. Once can simply update the XML description for the BT and the BtNavigator task server will use the new description.

## Behavior Tree nodes

A Behavior Tree consists of: control flow nodes, such as fallback, sequence, parallel, and decorator, as well as two execution nodes: condition and action nodes. Execution nodes are the leaf nodes of the tree. When a leaf node is ticked, the node does some work and it returns either SUCCESS, FAILURE or RUNNING.  The  The current Navigation2 software implements a few custom nodes, including Conditions and Actions. The user can also define and register additional node types that can then be used in BTs and the corresponding XML descriptions.

## Navigation Behavior Trees

The BT Navigator package has three sample XML-based descriptions of BTs.  These trees are [navigate_w_replanning.xml](behavior_trees/navigate_w_replanning.xml), [navigate_w_replanning_and_recovery.xml](behavior_trees/navigate_w_replanning_and_recovery.xml), and [auto_localization_w_replanning_and_recovery.xml](behavior_trees/auto_localization_w_replanning_and_recovery.xml).  The user may use any of these sample trees or develop a more complex tree which could better suit the user needs.

### Navigate with Replanning

[navigate_w_replanning.xml](behavior_trees/navigate_w_replanning.xml) implements basic navigation by continuously computing and updating the path at a rate of 1Hz and following the path at a rate of 10Hz.

```XML
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <RateController hz="1.0">
        <Fallback>
          <GoalReached/>
          <ComputePathToPose goal="${goal}"/>
        </Fallback>
      </RateController>
      <FollowPath path="${path}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

Navigate with replanning is composed of the following custom decorator, condition and action nodes:

#### Decorator Nodes
* RateController: A custom control flow node, which throttles down the tick rate.  This custom node has only one child and its tick rate is defined with a pre-defined frequency that the user can set.  This node returns RUNNING when it is not ticking its child. Currently, in the navigation, the `RateController` is used to tick the  `ComputePathToPose` and `GoalReached` node at 1 Hz.

#### Condition Nodes
* GoalReached: Checks the distance to the goal, if the distance to goal is less than the pre-defined threshold, the tree returns SUCCESS, which in that case the `ComputePathToPose` action node will not get ticked. 

#### Action Nodes
* ComputePathToPose: When this node is ticked, the goal will be placed on the blackboard which will be shared to the Behavior tree.  The bt action node would then utilizes the action server to send a request to the global planner to recompute the global path.  Once the global path is recomputed, the result will be sent back via action server and then the updated path will be placed on the blackboard.


The graphical version of this Behavior Tree:

<img src="./doc/navigation_w_replanning.png" title="Navigation with replanning Behavior Tree" align="middle">

The navigate with replanning BT first ticks the `RateController` node which specifies how frequently the `GoalReached` and `ComputePathToPose` should be invoked. Then the `GoalReached` nodes check the distance to the goal to determine if the `ComputePathToPose` should be ticked or not. The `ComputePathToPose` gets the incoming goal pose from the blackboard, computes the path and puts the result back on the blackboard, where `FollowPath` picks it up. Each time a new path is computed, the blackboard gets updated and then `FollowPath` picks up the new goal.

### Recovery Node
In this section, the recovery node is being introduced to the navigation package.

Recovery node is a control flow type node with two children.  It returns success if and only if the first child returns success. The second child will be executed only if the first child returns failure.  The second child is responsible for recovery actions such as re-initializing system or other recovery behaviors. If the recovery behaviors are succeeded, then the first child will be executed again.  The user can specify how many times the recovery actions should be taken before returning failure. The figure below depicts a simple recovery node.

<img src="./doc/RecoveryNode.png" title="" width="40%" align="middle">
<br/>

### Navigate with replanning and simple recovery actions

With the recovery node, simple recoverable navigation with replanning can be implemented by utilizing the [navigate_w_replanning.xml](behavior_trees/navigate_w_replanning.xml) and a sequence of recovery actions. Our custom behavior actions for recovery are:  `clearEntirelyCostmapServiceRequest` for both global and local costmaps and `spin`. A graphical version of this simple recoverable Behavior Tree is depicted in the figure below. 

<img src="./doc/navigation_w_replanning_and_recovery.png" title="" width="95%" align="middle">
<br/>


This tree is currently our default tree in the stack and the xml file is located here: [navigate_w_replanning_and_recovery.xml](behavior_trees/navigate_w_replanning_and_recovery.xml).


### AutoLocalization Behavior Tree
**Warning**: AutoLocalization actuates robot; currently, obstacle avoidance has not been integrated into this feature. The user is advised to not use this feature on a physical robot for safety reasons.  As of now, this feature should only be used in simulations.

[auto_localization.xml](behavior_trees/auto_localization.xml) Allows differential type robot to automatically localize its initial position when Nav Goal command is given to the robot without the initial pose.


Below is the `xml` representation of the tree.
```XML
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <FallbackStar name="root_AutoLocalization">
      <initialPoseReceived/>
      <SequenceStar name="doSelfLocalization">
        <RetryUntilSuccesful num_attempts="5" name="retry_client_request">
          <globalLocalizationServiceRequest/>
        </RetryUntilSuccesful>
        <RetryUntilSuccesful num_attempts="10" name="retry_localization">
          <Sequence>
            <Fallback>
              <IsLocalized/>
              <SequenceStar>
               <Spin/>
               <BackUp/>
               <Spin/>
             </SequenceStar>
            </Fallback>
            <IsLocalized/>
          </Sequence>
        </RetryUntilSuccesful>
      </SequenceStar>
    </FallbackStar>
  </BehaviorTree>
</root>
```

Image below depicts the graphical version of this Behavior Tree:
<img src="./doc/auto_localization.png" title="AutoLocalization branch of the Navigation Behavior Tree" width="60%" align="middle">

AutoLocalization branch is composed of the following condition and action nodes:

#### Condition Nodes
* initialPoseReceived: Checks `initial_pose` topic to determine if the initial pose has been received. Upon completion, this node returns either Success or Failure.
* isLocalized: Subscribes to `amcl_pose` and it checks the amcl pose covariance values for (x,y,rot) to determine if the robot is localized based on pre-defined tolerance. Upon completion, this node returns either Success or Failure.
#### Action Nodes
* globalLocalizationServiceRequest: Invokes a service call to amcl’s global localization to disperse particle cloud in free space of the map.
* Spin: Rotates the robot by sending angular velocity commands. This node currently is time based; the control based method has not been implemented yet. It returns either Success, Failure, or Running.
* BackUp: Backs up the robot by sending linear velocity commands in -x direction. This node currently is time based; the control based method has not been implemented yet. It returns either Success, Failure, or Running. Be advised that currently **obstacle avoidance** has not been integrated in the back up task yet.

The AutoLocalization branch starts by first determining if the initial robot pose has been received or not. If there is an initial pose, it will simply return Success and the AutoLocalization will not proceed. On the other hand, if initial pose is not received, it will return failure which causes the doAutoLocalization SequenceStar node to invoke. In this branch, first, the globalLocalizationServiceRequest gets ticked to generate uniform particle cloud in the free space of the map. Then, robot starts to spin and back up while simultaneously isLocalized node checks to determine if the robot is localized. If the robot location cannot be determined, the retry node will attempt the AutoLocalization process with pre-determined number of tries. Once the robot is localized, the tree will return success. If the robot is not localized by attempting all the retries, AutoLocalization branch will return Failure.

To run AutoLocalization branch, the `bt_navigator_params.yaml` file needs to be modified to include `auto_localization.xml` file. To run AutoLocalization with Recovery and Parallel Planning and Control, the `auto_localization_w_parallel_recovery.xml` needs to be included in the `bt_navigator_params.yaml` file.

Image below depicts the graphical version of the complete Navigation Task with AutoLocalization, Recovery, Parallel Planning and Control Behavior Tree:

<img src="./doc/AutoLocalization_w_recovery_parallel.png" title="Navigation Behavior Tree with AutoLocalization, Recovery, and Planning & Control" width="70%" align="middle">


## Open Issues

* **Schema definition and XML document validation** - Currently, there is no dynamic validation of incoming XML. The Behavior-Tree.CPP library is using tinyxml2, which doesn't have a validator. Instead, we can create a schema for the Mission Planning-level XML and use build-time validation of the XML input to ensure that it is well-formed and valid.
