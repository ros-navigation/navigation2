
# Navigation 2 with SLAM

This document explains how to use Navigation 2 with SLAM. The following steps show ROS 2 users how to generate occupancy grid maps and use Navigation 2 to move their robot around.

## Before the Tutorial

- [Install ROS 2](https://index.ros.org/doc/ros2/Installation/)
- Install Navigation 2
- Install SLAM package
- Install your robot package
- Source setup.bash in your ROS2 and Navigation 2 installation/build directory or workspaces

## Tutorial Steps

**1- (Robot)**  Launch the robot-related packages on your robot. 

Make sure /tf and /odom are being published.

- *Ex: Turtlebot 3*
            
       - ros2 launch turtlebot3_bringup robot.launch.py
    
**2- (Robot or PC)** [Launch Navigation](https://github.com/ros-planning/navigation2/tree/master/nav2_bringup)

This will bring up Navigation 2 without nav2_amcl and nav2_map_server. It is assumed that the SLAM node(s) will publish to /map topic and provide the map->odom transform.
             
      -  ros2 launch nav2_bringup nav2_navigation_launch.py

**3- (PC)** Launch SLAM package and RVIZ

Bring up your choice of SLAM implementation. Make sure it provides the map->odom transform and /map topic. Run Rviz and add the topics you want to visualize such as /map, /tf, /laserscan etc.

- *Ex. Cartographer*

*This is going to bring up Cartographer and Rviz*
   
      -  ros2 launch turtlebot3_cartographer cartographer.launch.py
          
**4- (PC)** Initialize the robot pose. Now, you can start sending goal poses using the goal tool in Rviz or via command line. The robot should start moving towards the goal pose, and the map should start appearing in Rviz.

- *Ex: move the robot 0.2 meters forward* 
    
      -  ros2 topic pub /move_base_simple/goal geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

**5- (PC)** Save the Map

        - ros2 run nav2_map_server map_saver -f ~/map


*TODO: Add the screenshots and videos.*
