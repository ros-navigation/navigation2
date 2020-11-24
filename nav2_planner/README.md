# Nav2 Planner

The Nav2 planner is a [planning module](../doc/requirements/requirements.md) that implements the `nav2_behavior_tree::ComputePathToPose` interface.

A planning module implementing the `nav2_behavior_tree::ComputePathToPose` interface is responsible for generating a feasible path given start and end robot poses. It loads a map of potential planner plugins like NavFn to do the path generation in different user-defined situations.
