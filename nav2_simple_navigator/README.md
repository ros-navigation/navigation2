# Simple Navigator

The Simple Navigator module implements a `nav2_tasks::NavigateToPose` task server. As such, its responsibility is to navigate the robot to the specified pose.

## Overview
 
The Simple Navigator implements NavigateToPose using two sub-tasks, `ComputePathToPose` and `FollowPath`, which are, in turn, task servers that implement the `nav2_tasks::ComputePathToPose` and `nav2_tasks::FollowPath` interfaces. It is conceptually similar to the MoveBase in the ROS 1 Navigation stack in that it coordinates the global planner (ComputePathToPose) and the local planner (FollowPath). 

The Simple Navigator sits in the task hierarchy, as follows:

<img src="https://github.com/ros-planning/navigation2/blob/master/nav2_tasks/doc/hierarchy.svg" width="400" title="Navigation Task Hiearchy">

The Simple Navigator module is meant to show how one can utilize (sub-)task clients in a direct way to implement a task. The BtNavigator module, an alternative implementation of NavigateToPose, is a drop-in replacement for the Simple Navigator and uses behavior trees to implement more complex behavior, including recovery.

## Implementation

In its [*execute* method](https://github.com/ros-planning/navigation2/blob/master/nav2_simple_navigator/src/simple_navigator.cpp), which directs the task server to begin its operation, the Simple Navigator invokes ComputePathToPose and FollowPath in sequence, managing the task clients for each directly. 
