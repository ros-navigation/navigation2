# Mission Executor

The Mission Executor module is a task server that coordinates other tasks. It can be used to direct the activities of multiple robots as well as other, non-robot tasks. As such, it would typically reside on a central orchestration computer. The Mission Executor module is implemented using [Behavior Trees](https://en.wikipedia.org/wiki/Behavior_tree). As detailed in [Michele Colledanchils's doctoral thesis](https://www.diva-portal.org/smash/get/diva2:1078940/FULLTEXT01.pdf), Behavior Trees are a Control Architecture (CA) initially used in the video game industry to control non-player characters and now applied to the control of autonomous robots. 

## Overview
 
The Mission Executor task server receives a **MissionPlan** and returns whether the plan was successfully executed, failed, or was canceled.

```C++
namespace nav2_tasks {

using ExecuteMissionCommand = nav2_msgs::msg::MissionPlan;
using ExecuteMissionResult = std_msgs::msg::Empty;

using ExecuteMissionTaskClient = TaskClient<ExecuteMissionCommand, ExecuteMissionResult>;
using ExecuteMissionTaskServer = TaskServer<ExecuteMissionCommand, ExecuteMissionResult>;

} //  namespace nav2_tasks
```

NOTE: An Empty message is used in the definition of the task client and server if there is no other data required.

The [mission plan message](https://github.com/ros-planning/navigation2/blob/master/nav2_msgs/msg/MissionPlan.msg) consists of a header and a mission plan string:

```
std_msgs/Header header
string mission_plan
```

The mission plan itself is an XML string that defines a Behavior Tree. For example, the following XML string specifies a sequence of three NavigateToPose actions:

```XML
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SequenceStar name="root">
      <NavigateToPose position="10;11;0.0" orientation="0.7071;0;0.7071;0"/>
      <NavigateToPose position="20;21;0.0" orientation="0.7071;0;0.7071;0"/>
      <NavigateToPose position="30;31;0.0" orientation="0.7071;0;0.7071;0"/>
    </SequenceStar>
  </BehaviorTree>
</root>

```

The mission plan module uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) to dynamically create a Behavior Tree from the input XML description. It then generates and executes the tree and returns the status code (SUCESSFUL, FAILED, CANCELED) to the task client.

## Incorporating Recovery Behaviors

The Behavior Tree can incorporate recovery actions by responding to conditions in the tree with other actions; a failure of one action could result in the execution of a recovery action, for example. 

## Creating Behavior Tree Nodes

To create a node that can be included in a Behavior Tree, there must first be an task server/client defined for it. For example, the NavigateToPose task is defined as follows:

```C++
namespace nav2_tasks
{

using NavigateToPoseCommand = geometry_msgs::msg::PoseStamped;
using NavigateToPoseResult = std_msgs::msg::Empty;

using NavigateToPoseTaskClient = TaskClient<NavigateToPoseCommand, NavigateToPoseResult>;
using NavigateToPoseTaskServer = TaskServer<NavigateToPoseCommand, NavigateToPoseResult>;

template<>
inline const char * getTaskName<NavigateToPoseCommand, NavigateToPoseResult>()
{
  return "NavigateToPoseTask";
}

}  // namespace nav2_tasks
```

Then, one can use the BtAction template to create an action node that can be included in a Behavior Tree. [ TODO: Once the Behavior Tree code is integrated describe the process of creating a Behavior Tree action ]

The Behavior Tree node automatically handles communication with the corresponding task server via a contained task client.

## Open Issues

* **Schema definition and XML document validation** - Currently, there is no dynamic validation of incoming XML. The Behavior-Tree.CPP library is using tinyxml2, which doesn't have a validator.
