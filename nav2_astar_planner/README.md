# A* Planner

The AStarPlanner is a [planning module](../doc/requirements/requirements.md) that implements the `nav2_tasks::ComputePathToPose` interface.

A planning module implementing the `nav2_tasks::ComputePathToPose` interface is responsible for generating a feasible path given start and end robot poses. It provides the equivalent functionality to a [GlobalPlanner](http://wiki.ros.org/nav_core#BaseGlobalPlanner) in ROS1 [MoveBase](http://wiki.ros.org/move_base).

## Status
Currently, AStarPlanner's core algorithm is a direct port from the ROS1 MoveBase [Navfn](http://wiki.ros.org/navfn) planner. The Navfn planning algorithm is based on the [Global Dynamic Window Approach](https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf). The Port for NavFn is in the `nav2_dijkstra_planner` package.

## Characteristics

The Navfn planner assumes a circular robot and operates on a costmap. While A* is faster than Dijkstra's, it does not promise optimality due to a heuristic used to try to drive the potential expansion towards the goal.

## Task Interface

The [Navigation System]((../doc/requirements/requirements.md)) is composed of three tasks: NavigateToPose, ComputePathToPose and FollowPathToPose.

The AStarPlanner implements the Task Server interface for ComputePathToPose, specifically derives from `nav2_tasks::ComputePathToPoseTaskServer`. The client to AStarPlanner is 'NavigateToPoseTask', which periodically sends requests to the path planner.

![alt text](../doc/design/NavigationSystemTasks.png "Navigation Tasks")
