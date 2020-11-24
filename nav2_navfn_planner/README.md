# Navfn Planner

The NavfnPlanner is a plugin for the Nav2 Planner server.

It provides the equivalent functionality to a [GlobalPlanner](http://wiki.ros.org/nav_core#BaseGlobalPlanner) in ROS1 [MoveBase](http://wiki.ros.org/move_base).

## Status
Currently, NavfnPlanner's core algorithm is a direct port from the ROS1 MoveBase [Navfn](http://wiki.ros.org/navfn) planner. The Navfn planning algorithm is based on the [Global Dynamic Window Approach](https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf).

## Characteristics

In Dijkstra mode (`use_astar = false`) Dijkstra's search algorithm is guaranteed to find the shortest path under any condition.
In A* mode (`use_astar = true`) A*'s search algorithm is not guaranteed to find the shortest path, however it uses a heuristic to expand the potential field towards the goal.

The Navfn planner assumes a circular robot and operates on a costmap.

## Next Steps
- Implement additional planners based on optimal control, potential field or other graph search algorithms that require transformation of the world model to other representations (topological, tree map, etc.) to confirm sufficient generalization. [Issue #225](http://github.com/ros-planning/navigation2/issues/225)
- Implement planners for non-holonomic robots. [Issue #225](http://github.com/ros-planning/navigation2/issues/225)
