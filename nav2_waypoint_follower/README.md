# Nav2 Waypoint Follower

The Nav2 waypoint follower is an example application of how to use the navigation action to complete some sort of orchestrated task. In this example, that task is to take a given set of waypoints and navigate to a set of positions in the order provided in the action request. The last waypoint in the waypoint array is the final position. It was built by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) while at [Samsung Research](https://www.sra.samsung.com/).

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-waypoint-follower.html) for additional parameter descriptions.

The package exposes the `follow_waypoints` action server of type `nav2_msgs/FollowWaypoints`.
 It is given an array of waypoints to visit, gives feedback about the current index of waypoint it is processing, and returns a list of waypoints it was unable to complete.

It also hosts a waypoint task executor plugin which can be used to perform custom behavior at a waypoint like waiting for user instruction, taking a picture, or picking up a box.

There is a parameterization `stop_on_failure` whether to stop processing the waypoint following action on a single waypoint failure. When false, it will continue onto the next waypoint when the current waypoint fails. The action will exist when either all the waypoint navigation tasks have terminated or when `stop_on_failure`, a single waypoint as failed.

## An aside on autonomy / waypoint following

The ``nav2_waypoint_follower`` contains a waypoint following program with a plugin interface for specific **task executors**.
This is useful if you need to go to a given location and complete a specific task like take a picture, pick up a box, or wait for user input.
It is a nice demo application for how to use Nav2 in a sample application.

However, it could be used for more than just a sample application.
There are 2 schools of thoughts for fleet managers / dispatchers.
- Dumb robot; smart centralized dispatcher
- Smart robot; dumb centralized dispatcher

In the first, the ``nav2_waypoint_follower`` is weakly sufficient to create a production-grade on-robot solution. Since the autonomy system / dispatcher is taking into account things like the robot's pose, battery level, current task, and more when assigning tasks, the application on the robot just needs to worry about the task at hand and not the other complexities of the system complete the requested task. In this situation, you should think of a request to the waypoint follower as 1 unit of work (e.g. 1 pick in a warehouse, 1 security patrole loop, 1 aisle, etc) to do a task and then return to the dispatcher for the next task or request to recharge. In this school of thought, the waypoint following application is just one step above navigation and below the system autonomy application.

In the second, the ``nav2_waypoint_follower`` is a nice sample application / proof of concept, but you really need your waypoint following / autonomy system on the robot to carry more weight in making a robust solution. In this case, you should use the ``nav2_behavior_tree`` package to create a custom application-level behavior tree using navigation to complete the task. This can include subtrees like checking for the charge status mid-task for returning to dock or handling more than 1 unit of work in a more complex task. Soon, there will be a ``nav2_bt_waypoint_follower`` (name subject to adjustment) that will allow you to create this application more easily. In this school of thought, the waypoint following application is more closely tied to the system autonomy, or in many cases, is the system autonomy.

Neither is better than the other, it highly depends on the tasks your robot(s) are completing, in what type of environment, and with what cloud resources available. Often this distinction is very clear for a given business case.
