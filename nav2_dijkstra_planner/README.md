# Dijkstra Planner
The DijkstraPlanner is an implementation of a [Planning module](../doc/requirements/requirements.md) (equivalent to GlobalPlanner in ROS1 [MoveBase](http://wiki.ros.org/move_base)).

The Planning module is responsible for generating a feasible path given start and end robot poses.

## Status
Currently, DijkstraPlanner is a direct port from ROS1 MoveBase [Navfn](http://wiki.ros.org/navfn) planner.

The Navfn planner assumes a circular robot and operates on a costmap to find a minimum cost plan from a start point to an end point in a grid. The navigation function is computed with Dijkstra's algorithm. The strategy is based on [High-Speed Navigation Using
the Global Dynamic Window Approach](https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf) Brock, O. and Oussama K. IEEE (1999)

## Task Interface

The [Navigation System]((../doc/requirements/requirements.md)) is composed of three tasks: NavigateToPose, ComputePathToPose and FollowPathToPose.

The DijkstraPlanner implements the Task Server interface for ComputePathToPose, specifically derives from `ComputePathToPoseTaskServer`. The client to DijkstraPlanner is 'NavigateToPoseTask', which periodically sends requests to the path planner.

![alt text](../doc/design/NavigationSystemTasks.png "Navigation Tasks")

## Testing
Currenty, component level tests are available that exercise the Planning Module Task Server API. The intention is to develop standard tests that can be used for any type of planner.

A PlannerTester node provides the world representation in the form of a costmap, sends a request to generate a path, and receives and checks the quality of the generated path.

As mentioned above, currently the world is represented as a costmap. Simplified versions of the world model and costmap are used for testing.

PlannerTester can sequentially pass random starting and goal poses and check the returned path for possible collision along the path.

Below is an example of the output from randomized testing. Blue spheres represent the starting locations, green, the goals. Red lines are the computed paths. Grey cells represent obstacles.

*Note: Currently robot size is 1x1 cells, no obstacle inflation is done on the costmap*

*Note: The Navfn algorithm sometimes fails to generate a path as you can see from the 'orphan' spheres.*

![alt text](../nav2_system_tests/src/planning/example_result.png "Output Example")


## Next Steps
- Refactor Navfn, currently difficult to modify/extend, use modern C++.
- Develop/Integrate other planning modules based on more recent algorithms that test other world representations, robot configurations, etc.

Interested on collaborating?
Feel free to contact Carlos Orduno, carlos.a.orduno@intel.com