# Nav2 Loopback Simulator 

The Nav2 Loopback Simulator package offers a simplified Gazebo replacement, focusing on sending robot velocity commands to TF as odometry, facilitating high-level behavioral testing without the complexity of full simulation.

## Usage

`ros2 launch nav2_loopback_sim loopback_sim_launch.py map:=/path_to_ros_ws/navigation2/nav2_bringup/bringup/maps/turtlebot3_world.yaml`