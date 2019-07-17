**Warning**: As with the rest of `nav2`, this package is still in development and only works with Turtlebot 3 at the moment. Currently collision avoidance has not been integrated. The user is advised to not use this feature on a physical robot for safety reasons.  As of now, this feature should only be used in simulations.

---

# Motion Primitives

The `nav2_motion_primitives` package implements, as the name suggests, a module for executing simple controlled robot movements such as rotating on its own axis or moving linearly.

The package defines:
- A `MotionPrimitive` template which is used as a base class to implement specific primitives.
- The `BackUp`, `Spin` and `Stop` primitives.

## Overview

*Motion primitives* define simple predictable movements that components can leverage for defining more complex behavior. For example, `nav2` uses motion primitives for executing recovery behaviors, such as the ones defined on the [BtNavigator](../nav2_bt_navigator/README.md##Recovery-Behavior-Trees).

Currently the package provides the following primitives:
- **Spin** performs an in-place rotation by a given angle.
- **Back Up** performs an linear translation by a given distance.
- **Stop** brings the robot to a stationary state.

## Implementation

The module is implemented as a single node containing multiple primitives and follows the `nav2` [task hierarchy](../nav2_behavior_tree/README.md#Overview). Each primitive is defined as a `nav2_task` with corresponding *command* and *result* message definitions.

The `MotionPrimitive` base class manages the task server, provides a robot interface and calls the primitive's update functions.

To gain insight into the package lets go over how to implement and execute a new motion primitive.

### Defining a primitive

In this section we'll go over how to define a new primitive and implement the corresponding motion.

The first step is to provide the [task definition](../nav2_behavior_tree/README.md) inside the `nav2_behavior_tree` package. For example, lets define a `SomePrimitive` task interface, i.e. the types of messages to use for the command and result, as well as the client and server.

```cpp
namespace nav2_behavior_tree
{

using SomePrimitiveCommand = geometry_msgs::msg::Point;
using SomePrimitiveResult = std_msgs::msg::Empty;

using SomePrimitiveTaskClient = TaskClient<SomePrimitiveCommand, SomePrimitiveResult>;
using SomePrimitiveTaskServer = TaskServer<SomePrimitiveCommand, SomePrimitiveResult>;

template<>
inline const char * getTaskName<SomePrimitiveCommand, SomePrimitiveResult>()
{
  return "SomePrimitiveTask";
}

}  // namespace nav2_behavior_tree
```

For this example we arbitrarily pick `geometry_msgs::msg::Point` and `std_msgs::msg::Empty` as message types for command and result.

Next we define the class for our new primitive. This class should derive from `MotionPrimitive` and use the command and result messages defined on the corresponding task.

```cpp
class SomePrimitive : public MotionPrimitive<nav2_behavior_tree::SomePrimitiveCommand, nav2_behavior_tree::SomePrimitiveResult>
```

On the implementation of `SomePrimitive` all we do is override `onRun` and `onCycleUpdate`.

```cpp
using nav2_behavior_tree

TaskStatus SomePrimitive::onRun(const SomePrimitiveCommand::SharedPtr command)
{
    /* onRun code */
}

TaskStatus SomePrimitive::onCycleUpdate(SomePrimitiveResult & result)
{
    /* onCycleUpdate code */
}
```

The `onRun` method is the entry point for the primitive and here we should:
- Catch the command.
- Perform checks before the main execution loop.
- Possibly do some initialization.
- Return a `nav2_behavior_tree::TaskStatus` given the initial checks.

The `onCycleUpdate` method is called periodically until it returns `FAILED` or `SUCCEEDED`, here we should:
- Set the robot in motion.
- Perform some unit of work.
- Check if the robot state, determine if work completed
- Return a `nav2_behavior_tree::TaskStatus`.

### Defining the primitive's client

Primitives use the `nav2_behavior_tree` interface, so we need to define the task client:

```cpp
nav2_behavior_tree::TaskClient<SomePrimitiveCommand, SomePrimitiveResult> some_primitive_task_client;
```

To send requests we create the command and sent it over the client:

```cpp
SomePrimitiveCommand::SharedPtr command;
// Fill command
some_primitive_task_client.sendCommand(command)
```

### (optional) Define the Behavior Tree action node

For using motion primitivies within a behavior tree such as [bt_navigator](../nav2_bt_navigator/README.md##Navigation-Behavior-Trees), then a corresponding *action node* needs to be defined. Checkout `nav2_behavior_tree` for examples on how to implement one.

## Plans

- Check for collision before executing a primitive. Issues [379](https://github.com/ros-planning/navigation2/issues/379) and [533](https://github.com/ros-planning/navigation2/issues/533).
- Remove the stop primitive, move the funcionality to the robot class. Issue [575](https://github.com/ros-planning/navigation2/issues/575)
- Consider moving `nav2_motion_primitives` altogether to the `nav2_robot` package. Issue [378](https://github.com/ros-planning/navigation2/issues/378).
- Depending on the feedback from the community we might want to develop this package to include a wide variety of primitives (arcs) to support all kinds of task, navigation (lattice-based), docking, etc.
- Define smooth transitions between motions. Issue [411](https://github.com/ros-planning/navigation2/issues/411).
- Make the existing motion primitives configurable for other robots.

Refer to Github for an up-to-date [list](https://github.com/ros-planning/navigation2/issues?q=is%3Aopen+is%3Aissue+label%3Anav2_motion_primitives).
