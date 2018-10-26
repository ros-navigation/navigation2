# Bringup

## Before starting:
- Build ROS2 from master branch
- If working with Ubuntu 16, install [Gazebo 9](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
  - In step #3:

    ```$ sudo apt-get install gazebo9-common libsdformat6 gazebo9 libgazebo-dev```

  - You might want to cleanup prior Gazebo installations:

    ```$ sudo apt autoremove```

- Build gazebo_ros_pkgs from ros2 branch

## Driving the Gazebo differential drive robot

  To run:

      $ gazebo <dash><dash>verbose <path_to>/gazebo_ros_simplebot.world

  Try sending commands:

      $ ros2 topic pub /nav2/cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}}' -1
      $ ros2 topic pub /nav2/cmd_vel geometry_msgs/Twist '{angular: {z: 0.1}}' -1

  Try listening to odometry:

      $ ros2 topic echo /nav2/odom

  Try listening to TF:

      $ ros2 run tf2_ros tf2_echo odom base_link
      $ ros2 run tf2_ros tf2_echo base_link right_wheel
      $ ros2 run tf2_ros tf2_echo base_link left_wheel

  Open RVIZ:

      $ ros2 run rviz2 rviz2

  On RViz, add the following topics:

      * /ray/pointcloud2
      * /ray/pointcloud
      * /ray/laserscan

  *Note that the laser scans are not working on RViz due to https://github.com/ros2/rviz/issues/332*

  Provide a transform for the laser sensor:

      $ ros2 run tf2_ros static_transform_publisher 0.75 0 0.484 0 0 0 base_link ray_link

  On RViz, change the Fixed Frame" to ```ray_link```

  Echo the range topics, i.e.

      $ ros2 topic echo /ray/range

  ## Next Steps
- Fix transforms for laser sensor
- Add transform for imu?
- Fix topic names
- Verify the output of the imu
- Reduce the size of the robot
- Create launch file? use empty_world.launch.py as an example
