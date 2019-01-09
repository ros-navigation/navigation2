# Mission Executor

The Mission Executor module is a task server that coordinates other tasks. It can be used to direct the activities of multiple robots as well as other, non-navigation and non-robot tasks. This module would typically reside on a central orchestration computer and direct the operation of remote robots. 

## Overview

The Mission Executor module is implemented using [Behavior Trees](https://en.wikipedia.org/wiki/Behavior_tree). As detailed in [Michele Colledanchils's doctoral thesis](https://www.diva-portal.org/smash/get/diva2:1078940/FULLTEXT01.pdf), Behavior Trees are a Control Architecture (CA) initially used in the video game industry to control non-player characters and now applied to the control of autonomous robots. 

The Mission Executor task server receives a **MissionPlan** and returns whether the plan was successfully executed, failed, or was canceled. The ExecuteMissionTask is defined as follows:

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

## Invoking the MissionExecutor

The MissionExecutor is an implementation of the ExecuteMissionTaskServer interface, so the corresponding ExecuteMissionTaskClient is used to communicate with the task server, like this:

```C++
// Demonstrate using a task client to invoke the MissionExecutor task
void invokeMission(rclcpp::Node::SharedPtr node, std::string mission_plan)
{
  // Create a task client for this particular kind of task
  auto task_client_ = std::make_unique<nav2_tasks::ExecuteMissionTaskClient>(node);

  // Create the input and output 
  auto cmd = std::make_shared<nav2_tasks::ExecuteMissionCommand>();
  auto result = std::make_shared<nav2_tasks::ExecuteMissionResult>();

  // Set the mission_plan field to the XML input
  cmd->mission_plan = mission_plan;

  // Send this message to the task server
  task_client_->sendCommand(cmd);

  // Loop until the tasks is complete
  for (;; ) {
    // Get the status of the remote task server
    TaskStatus status = task_client_->waitForResult(result, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        printf("Task succeeded\n");
        return;

      case TaskStatus::FAILED:
        printf("Task failed\n");
        return;

      case TaskStatus::CANCELED:
        printf("Task cancelled\n");
        return;

      case TaskStatus::RUNNING:
        // Continue waiting for task to complete
        break;

      default:
        throw std::logic_error("Invalid status value");
    }
  }
}
```

## Open Issues

* **Schema definition and XML document validation** - Currently, there is no dynamic validation of incoming XML. The Behavior-Tree.CPP library is using tinyxml2, which doesn't have a validator. Instead, we can create a schema for the Mission Planning-level XML and use build-time validation of the XML input to ensure that it is well-formed and valid.
