This simple test verifies that a node is subsribed to /scan topic and then it checks to see if amcl_pose msg is published with a valid number (i.e. !NaN).

To run the test, Gazebo and ros1_bridge needs to be running.

1. roslaunch turtlebot_gazebo turtlebot_world.launch
2. ros2 run ros1_bridge dynamic_bridge
