# Mission Executor

The Mission Executor module is a task server that coordinates other tasks. It can be used to direct the activities multiple robots as well as other, non-robot tasks.

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

The mission plan itself is an XML string that defines the behavior tree. For example, the following XML string specifies a sequence of three NavigateToPose actions:

```XML
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SequenceStar name="root">
      <NavigateToPoseAction position="10;11;0.0" orientation="0.7071;0;0.7071;0"/>
      <NavigateToPoseAction position="20;21;0.0" orientation="0.7071;0;0.7071;0"/>
      <NavigateToPoseAction position="30;31;0.0" orientation="0.7071;0;0.7071;0"/>
    </SequenceStar>
  </BehaviorTree>
</root>

```

The mission plan module uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) to dynamically create a behavior tree from the input XML description. It then generates and executes the tree and returns the status code (SUCESSFUL, FAILED, CANCELED) to the task client.

## Incorporating Recovery Behaviors

The behavior tree can incorporate recovery actions by responding to conditions in the tree with other actions; a failure of one action could result in the execution of a recovery action, for example. 

## Creating Behavior Tree Nodes

To create a node that can be included in a behavior tree, there must first be an task server/client defined for it. For example, the NavigateToPose task is defined as follows:

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

Then, one can use the BtAction template to create an action node that can be included in a behavior tree. [ TODO: Once the behavior tree code is integrated describe the process of creating a behavior tree action ]

The behavior tree node automatically handles communication with the corresponding task server via a contained task client.

## Open Issues

* **Schema definition and XML document validation**

  + Currently, there is no dynamica validation of incoming XML. The Behavior-Tree.CPP library is using tinyxml2, which doesn't have a validator.
