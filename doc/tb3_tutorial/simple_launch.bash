#!/bin/bash


#*******************Changes in This Part***********************#
#
# define your workspace names
# export parent_ws_name=<workspace_name>
#
export parent_ws_name=ros2_all_ws
export ros2_ws_name=ros2_ws
export tb3_ws_name=turtlebot3_ws
export nav2_ws_name=navigation2_ws
export nav_dep_ws_name=navstack_dependencies_ws
#
# set the time delay between commands 
# in second
#
sleep_time=10
#
# robot type
#
export robot=waffle
#
#************NO Change Is Required In This Part*****************#
#*********Change Only If You Know What You Are Doing*************#
#
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/"$USER"/"$parent_ws_name"/"$tb3_ws_name"/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=$robot;

gnome-terminal --working-directory=/home --command 'bash -c 
"
echo Gazebo is launching;
source /home/"$USER"/"$parent_ws_name"/"$tb3_ws_name"/install/setup.bash; 
source /home/"$USER"/"$parent_ws_name"/"$ros2_ws_name"/install/setup.bash; 
source /home/"$USER"/"$parent_ws_name"/"$nav_dep_ws_name"/install/setup.bash; 
source /home/"$USER"/"$parent_ws_name"/"$nav2_ws_name"/install/setup.bash; 
killall gzserver; 
gazebo --verbose -s libgazebo_ros_init.so /home/"$USER"/"$parent_ws_name"/"$tb3_ws_name"/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_worlds/"$robot.model"; 
exec bash"'

sleep $sleep_time

gnome-terminal --working-directory=/home --command 'bash -c 
"
echo Turtlebot 3 is launching;
source /home/"$USER"/"$parent_ws_name"/"$tb3_ws_name"/install/setup.bash; 
source /home/"$USER"/"$parent_ws_name"/"$ros2_ws_name"/install/setup.bash; 
source /home/"$USER"/"$parent_ws_name"/"$nav_dep_ws_name"/install/setup.bash; 
source /home/"$USER"/"$parent_ws_name"/"$nav2_ws_name"/install/setup.bash; 
ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py use_sim_time:=True; 
exec bash"'

sleep $sleep_time

gnome-terminal --working-directory=/home --command 'bash -c 
"
echo Navigation 2 is launching;
source /home/"$USER"/"$parent_ws_name"/"$tb3_ws_name"/install/setup.bash; 
source /home/"$USER"/"$parent_ws_name"/"$ros2_ws_name"/install/setup.bash; 
source /home/"$USER"/"$parent_ws_name"/"$nav_dep_ws_name"/install/setup.bash; 
source /home/"$USER"/"$parent_ws_name"/"$nav2_ws_name"/install/setup.bash; 
ros2 launch nav2_bringup nav2_bringup_launch.py use_sim_time:=True autostart:=False     map:=/home/"$USER"/"$parent_ws_name"/"$nav2_ws_name"/install/nav2_bringup/share/nav2_bringup/launch/turtlebot3_world.yaml;
exec bash"'

sleep $sleep_time

gnome-terminal --working-directory=/home --command 'bash -c 
"
echo Rviz is launching;
source /home/"$USER"/"$parent_ws_name"/"$tb3_ws_name"/install/setup.bash; 
source /home/"$USER"/"$parent_ws_name"/"$ros2_ws_name"/install/setup.bash; 
source /home/"$USER"/"$parent_ws_name"/"$nav_dep_ws_name"/install/setup.bash; 
source /home/"$USER"/"$parent_ws_name"/"$nav2_ws_name"/install/setup.bash; 
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/nav2_default_view.rviz; 
exec bash"'
