# Dijkstra Planner
The DijkstraPlanner is an implementation of a [Planning module](../doc/requirements/requirements.md) (equivalent to GlobalPlanner in ROS1 MoveBase).

The Planning module is responsible for generating a feasible path given start and end robot poses.

## Status
Currently, DijkstraPlanner is a direct port from MoveBase Navfn planner.

**continue here**
 (which is based on Dijkstra algorithm).

We found some bugs.

Reasons for selecting Navfn over other planners

## Task Client

## Why task

## Testing
Component level tests are available that exercise the Planning Module (**task name**) API. The intention is to develop standard tests that can be used for any type of planner.

A PlannerTester node provides the world representation in the form of a costmap, sends a request to generate a path, and receives and checks the quality of the generated path.

As mentioned above, currently the world is represented as a costmap. Simplified versions of the world model and costmap are used for testing.

PlannerTester can sequentially pass random starting and goal poses and check the returned path for possible collision along the path.

Below is an example of the output from randomized testing. Blue spheres represent the starting locations, green, the goals. Red lines are the computed paths. Grey cells represent obstacles.

*Note: Currently robot size is 1x1 cells, no obstacle inflation is done on the costmap*

*Note: The Navfn algorithm sometimes fails to generate a path as you can see from the 'orphan' spheres.*

![alt text](../nav2_system_tests/src/planning/example_result.png "Output Example")

## Open Issues

## Next Steps
- Refactor navfn, use modern C++
- Implement a second planning module that test other world representations, robot configurations, etc.
- Unit Tests - in progress

Interested on collaborating?
Feel free to contact Carlos Orduno, carlos.a.orduno@intel.com