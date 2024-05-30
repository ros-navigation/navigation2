# Nav2 System Tests

This is a 'top level' system test which will use Gazebo to simulate a Robot moving from an known initial starting position to a goal pose. 

## To run the test
First, you must build Nav2 including this package:
```
colcon build --symlink-install
```
Then you can run all the system tests:
```
colcon test --packages-select nav2_system_tests
```
Output results will go to the screen, and will be logged to the "log/latest_test/nav2_system_tests/" path relative to where colcon test was run.

To run just the bt_navigator test:
```
cd build/nav2_system_tests
ctest -V bt_navigator$
```

To loop over the bt_navigator test, a script has been provided:
```
nav2_system_tests/scripts/ctest_loop.bash -c <# loops> -o <path/to/summary/filename.txt> -l <path/to/store/failing/logfiles.log> -d <dds to use>
```

Example (loop 100 times using fastrtps):
```
nav2_system_tests/scripts/getopt_ctest_loop.bash -c 100 -o /home/robot/data/results.txt -l /home/robot/data/results.log -d rmw_fastrtps_cpp
```

## Notes: (updated Aug 2019)
 * This currently uses a turtlebot3 robot model, world and map.
 * The test normally takes 1-2 minutes to run, with a timeout of 2 minutes

## Future Work
 * Add additional goal poses if the first one successfully passes
 * Remove the dependency on the turtlebot3 model and map by adding a simple / dummy robot and creating a world and map
