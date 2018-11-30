# Localization Testing

The intention of the localization test is to ensure robot’s pose and transforms are available.

Currently, only a simple test that checks the `initialpose` has been implemented.  The `test_localization` module publishes an initial pose on `initialpose` topic and then it listens to `amcl_pose` topic. If the `amcl_pose` is similar to `initial pose` within a predefined tolerance the test passes.

Before building GAZEBO_MODEL_PATH variable needs to be set and then ctest command can be used to initiate the test.  

1. Before building open a terminal and set `export GAZEBO_MODEL_PATH= /turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models`

2. `colcon build --symlink-install`

3. After building, from /build/nav2_system_tests directory run: `ctest -V -R test_localization`

## Future Plan
Once rosbag functionality becomes available, this test can be extended to utilize a recorded trajectory with map and scan data to monitor the `amcl_pose` and `transforms` without a need to run Gazebo and map_server.
