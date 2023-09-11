# Nav2 Simple (Python3) Commander

## Overview

The goal of this package is to provide a "navigation as a library" capability to Python3 users. We provide an API that handles all the ROS2-y and Action Server-y things for you such that you can focus on building an application leveraging the capabilities of Nav2. We also provide you with demos and examples of API usage to build common basic capabilities in autonomous mobile robotics.

This was built by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) at [Samsung Research](https://www.sra.samsung.com/), with initial prototypes being prepared for the Keynote at the [2021 ROS Developers Day](https://www.theconstructsim.com/ros-developers-day-2021/) conference (code can be found [here](https://github.com/SteveMacenski/nav2_rosdevday_2021)).

![](media/readme.gif)

## API

See its [API Guide Page](https://navigation.ros.org/commander_api/index.html) for additional parameter descriptions.

The methods provided by the basic navigator are shown below, with inputs and expected returns. If a server fails, it may throw an exception or return a `None` object, so please be sure to properly wrap your navigation calls in try/catch and check results for `None` type.

New as of September 2023: the simple navigator constructor will accept a `namespace` field to support multi-robot applications or namespaced Nav2 launches.

| Robot Navigator Method            | Description                                                                |
| --------------------------------- | -------------------------------------------------------------------------- |
| setInitialPose(initial_pose)      | Sets the initial pose (`PoseStamped`) of the robot to localization.        |
| goThroughPoses(poses, behavior_tree='') | Requests the robot to drive through a set of poses (list of `PoseStamped`).|
| goToPose(pose, behavior_tree='')  | Requests the robot to drive to a pose (`PoseStamped`).                     |
| followWaypoints(poses)            | Requests the robot to follow a set of waypoints (list of `PoseStamped`). This will execute the specific `TaskExecutor` at each pose.   |
| followPath(path, controller_id='', goal_checker_id='') | Requests the robot to follow a path from a starting to a goal `PoseStamped`, `nav_msgs/Path`.     |
| spin(spin_dist=1.57, time_allowance=10)   | Requests the robot to performs an in-place rotation by a given angle.      |
| backup(backup_dist=0.15, backup_speed=0.025, time_allowance=10) | Requests the robot to back up by a given distance.         |
| cancelTask()                       | Cancel an ongoing task request.|
| isTaskComplete()                   | Checks if task is complete yet, times out at `100ms`.  Returns `True` if completed and `False` if still going.                  |
| getFeedback()                     | Gets feedback from task, returns action server feedback object. |
| getResult()				        | Gets final result of task, to be called after `isTaskComplete` returns `True`. Returns action server result object. |
| getPath(start, goal, planner_id='', use_start=False) | Gets a path from a starting to a goal `PoseStamped`, `nav_msgs/Path`.      |
| getPathThroughPoses(start, goals, planner_id='', use_start=False) | Gets a path through a starting to a set of goals, a list of `PoseStamped`, `nav_msgs/Path`. |
| smoothPath(path, smoother_id='', max_duration=2.0, check_for_collision=False) | Smooths a given `nav_msgs/msg/Path` path. |
| changeMap(map_filepath)           | Requests a change from the current map to `map_filepath`'s yaml.           |
| clearAllCostmaps()                | Clears both the global and local costmaps.                                 |
| clearLocalCostmap()               | Clears the local costmap.                                                  |
| clearGlobalCostmap()              | Clears the global costmap.                                                 |
| getGlobalCostmap()                | Returns the global costmap, `nav2_msgs/Costmap`                            |
| getLocalCostmap()                 | Returns the local costmap, `nav2_msgs/Costmap`                             |
| waitUntilNav2Active(navigator='bt_navigator, localizer='amcl') | Blocks until Nav2 is completely online and lifecycle nodes are in the active state. To be used in conjunction with autostart or external lifecycle bringup. Custom navigator and localizer nodes can be specified  |
| lifecycleStartup()                | Sends a request to all lifecycle management servers to bring them into the active state, to be used if autostart is `false` and you want this program to control Nav2's lifecycle. |
| lifecycleShutdown()               | Sends a request to all lifecycle management servers to shut them down.     |
| destroyNode()                     | Releases the resources used by the object.                                 |

A general template for building applications is as follows:

``` python3

from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

rclpy.init()

nav = BasicNavigator()
...
nav.setInitialPose(init_pose)
nav.waitUntilNav2Active() # if autostarted, else use `lifecycleStartup()`
...
path = nav.getPath(init_pose, goal_pose)
smoothed_path = nav.smoothPath(path)
...
nav.goToPose(goal_pose)
while not nav.isTaskComplete():
	feedback = nav.getFeedback()
	if feedback.navigation_duration > 600:
		nav.cancelTask()
...
result = nav.getResult()
if result == TaskResult.SUCCEEDED:
    print('Goal succeeded!')
elif result == TaskResult.CANCELED:
    print('Goal was canceled!')
elif result == TaskResult.FAILED:
    print('Goal failed!')
```

## Usage of Demos and Examples

Make sure to install the `aws_robomaker_small_warehouse_world` package or build it in your local workspace alongside Nav2. It can be found [here](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world). The demonstrations, examples, and launch files assume you're working with this gazebo world (such that the hard-programmed shelf locations and routes highlighting the API are meaningful).

Make sure you have set the model directory of turtlebot3 simulation and aws warehouse world to the `GAZEBO_MODEL_PATH`. There are 2 main ways to run the demos of the `nav2_simple_commander` API.

### Automatically

The main benefit of this is automatically showing the above demonstrations in a single command for the default robot model and world. This will make use of Nav2's default robot and parameters set out in the main simulation launch file in `nav2_bringup`.

``` bash
# Launch the launch file for the demo / example
ros2 launch nav2_simple_commander  security_demo_launch.py
```

This will bring up the robot in the AWS Warehouse in a reasonable position, launch the autonomy script, and complete some task to demonstrate the `nav2_simple_commander` API.

### Manually

The main benefit of this is to be able to launch alternative robot models or different navigation configurations than the default for a specific technology demonstration. As long as Nav2 and the simulation (or physical robot) is running, the simple python commander examples / demos don't care what the robot is or how it got there. Since the examples / demos do contain hard-programmed item locations or routes, you should still utilize the AWS Warehouse. Obviously these are easy to update if you wish to adapt these examples / demos to another environment.

``` bash
# Terminal 1: launch your robot navigation and simulation (or physical robot). For example
ros2 launch nav2_bringup tb3_simulation_launch.py world:=/path/to/aws_robomaker_small_warehouse_world/.world map:=/path/to/aws_robomaker_small_warehouse_world/.yaml

# Terminal 2: launch your autonomy / application demo or example. For example
ros2 run nav2_simple_commander demo_security
```

Then you should see the autonomy application running!

## Examples

The `nav2_simple_commander` has a few examples to highlight the API functions available to you as a user:

- `example_nav_to_pose.py` - Demonstrates the navigate to pose capabilities of the navigator, as well as a number of auxiliary methods.
- `example_nav_through_poses.py` - Demonstrates the navigate through poses capabilities of the navigator, as well as a number of auxiliary methods.
- `example_waypoint_follower.py` - Demonstrates the waypoint following capabilities of the navigator, as well as a number of auxiliary methods.
- `example_follow_path.py` - Demonstrates the path following capabilities of the navigator, as well as a number of auxiliary methods such as path smoothing.
## Demos

The `nav2_simple_commander` has a few demonstrations to highlight a couple of simple autonomy applications you can build using the `nav2_simple_commander` API:

- `demo_security.py` - A simple security robot application, showing how to have a robot follow a security route using Navigate Through Poses to do a patrol route, indefinitely. 
- `demo_picking.py` - A simple item picking application, showing how to have a robot drive to a specific shelf in a warehouse to either pick an item or have a person place an item into a basket and deliver it to a destination for shipping using Navigate To Pose.
- `demo_inspection.py` - A simple shelf inspection application, showing how to use the Waypoint Follower and task executors to take pictures, RFID scans, etc of shelves to analyze the current shelf statuses and locate items in the warehouse.
