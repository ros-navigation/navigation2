# Turtlebot3 Waffle Gazebo Model with Depth Camera Plugin
This is the Gazebo ROS2 camera plugin enabled version of Turtlebot 3 Waffle Gazebo model.

- RGB Image
- Depth Image
- Point Cloud

Tested using Gazebo 9 and ROS2 Dashing.

# Requirements

- [Install ROS2](https://index.ros.org/doc/ros2/Installation/Dashing/)
- Install Gazebo
    - ```curl -sSL http://get.gazebosim.org | sh```
- Install Gazebo ROS2 Packages
    - ```sudo apt install ros-<ros2_distro>-gazebo-*```
- Source setup.bash in the ROS2 installation directory
    - ```source /opt/ros/<ros2_distro>/setup.bash```
- Clone the repository and copy turtlebot3_waffle into ```.gazebo/models``` directory. It is located in the ```/home``` directory or where you store your model files such ```turtlebot3_simulation/turtlebot3_gazebo/models```
- Use the urdf with robot state publihser or replace the description file in ```turtlebot3_description/urdf``` with this urdf file.

# Published Topics

- /intel_realsense_r200_depth/camera_info_raw
- /intel_realsense_r200_depth/depth/camera_info
- /intel_realsense_r200_depth/depth/image_raw
- /intel_realsense_r200_depth/image_raw
- /intel_realsense_r200_depth/points
- /intel_realsense_r200_rgb/camera_info
- /intel_realsense_r200_rgb/image_raw

# Camera Frame Names: 

- realsense_depth_frame
- raelsense_rgb_frame

## Point Cloud, Turtlebot 3 Robot Model, and Map in Rviz
![gz_realsense.png](https://github.com/mlherd/ros2_turtlebot3_waffle_intel_realsense/blob/master/pics/rviz_map_point_cloud.png?raw=true)

## Turtlebot 3 with Intel RealSense in Turlebot World in Gazebo
![gz_realsense.png](https://raw.githubusercontent.com/mlherd/ros2_turtlebot3_waffle_intel_realsense/master/pics/gz_realsense.png)

## Point Cloud view in Rviz
![gz_realsense.png](https://raw.githubusercontent.com/mlherd/ros2_turtlebot3_waffle_intel_realsense/master/pics/rviz_point_cloud.png)
