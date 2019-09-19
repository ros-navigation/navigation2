**Navigation 2 with Cartogtographer**

This document explains how to use Navigation 2  with Cartographer. This allows ROS 2 users to generate maps and navigate at the same time. In other words, the user can easily navigate their robot in an unknown environment by sending target poses.

- Install ROS 2
- Install Navigation 2
- Install Cartographer
- Install your robot packages. 
- Source setup.bash in your ROS2 and Navigation 2 installation/build directory or workspaces
- Setting the ROS Domain ID variable is recommended. Ex:
    - `export ROS_DOMAIN_ID=30`

## Tutotrial Steps

**1- (Robot)**  Bring up the robot-related packages on your robot. 
Make sure /tf and /odom are being published.
         - Ex: Turtlebot 3
            
       - ros2 launch turtlebot3_bringup robot.launch.py
    
**2- (Robot or PC)** Bring up Navigation
This will bring up Navigation 2 without nav2_amcl and nav2_map_server. Cartographer will publish to /map topic and do localization.
             
      -  ros2 launch nav2_bringup nav2_navigation_launch.py

**3- (PC)** Bring up Cartographer
   This is going to bring up Cartographer and Rviz.
   
      -  ros2 launch turtlebot3_cartographer cartographer.launch.py
          
**4- (PC)** Initialize robot pose and send goal poses using the goal tool in Rviz or via command line. 
    Ex: move robot 0.2 meters forward 
    
      -  ros2 topic pub /move_base_simple/goal geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

- **Save the Map**

        - ros2 run nav2_map_server map_saver -f ~/map


*TODO: Add the screenshots and videos.*
