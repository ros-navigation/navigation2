# Navigation2 System Tests

This is a 'top level' system test which will use Gazebo to simulate a Robot moving from an known initial starting position to a goal pose. 

## To run the test
First, build the package
```
colcon build --symlink-install
```
Then you can run the test:
```
colcon test --packages-select nav2_system_tests
```
Output results will go to the screen, and will be logged to the "log/latest_test/nav2_system_tests/" path relative to where colcon test was run.

## Notes: (Dec 18 Crystal release)
 * This currently uses a turtlebot3 robot model, world and map.
 * The test normally takes 1-2 minutes to run, with a timeout of 2 minutes
 * Due to current issues with the Navigation2 initialization, the bringup can sometimes require a retry, so the test automatically internally retries sending an initial pose and goal pose if it doesn't reach the goal within a default 30 second time from when the goal pose was sent
 * The test is currently using the 'simple_navigator' controller
 
## Future Work
 * Add an additional test for the 'bt_navigator' controller
 * Add additional goal poses if the first one successfully passes
 * Remove the dependency on the turtlebot3 model and map by adding a simple / dummy robot and creating a world and map