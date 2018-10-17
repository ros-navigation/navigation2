# Mission Executor

The Mission Executor module is a task server (simple action server) that coordinates other tasks. It can be used to direct the activities multiple robots as well as other, non-robot tasks.

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

The [mission plan message](../nav_msgs/msg/MissionPlan.msg) consists of a header and a mission plan string:

```
std_msgs/Header header
string mission_plan
```

The mission plan itself is an XML string that defines the behavior tree. For example, the following XML string specifies a sequence of three NavigateToPose actions:

```XML
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SequenceStar name="root">
      <NavigateToPoseAction position="10;11;0.0" orientation="1;2;3;4"/>
      <NavigateToPoseAction position="20;21;0.0" orientation="5;6;7;8"/>
      <NavigateToPoseAction position="30;31;0.0" orientation="9;1;2;3"/>
    </SequenceStar>
  </BehaviorTree>
</root>

```

The mission plan module uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) to dynamically create a behavior tree from the input XML description. It then generates and executes the tree and returns the status code (SUCESSFUL, FAILED, CANCELED) to the task client. 

## Creating Behavior Tree Nodes

To create a node that can be included in a behavior tree, there must first be an task server/client defined for it. To define a task server and client one defines the command and result messages for that task, the templates for the TaskClient and TaskServer, and the name for the task. For example, the NavigateToPose task is defined as follows:

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

Then, one can use the BtAction template. 

[ TODO: Once the behavior tree code is integrated describe the process of creating a behavior tree action ]

## Open Issues

* **Schema definition and XML document validation**

  + Currently, there is no dynamica validation of incoming XML. The Behavior-Tree.CPP library is using tinyxml2, which doesn't have a validator.