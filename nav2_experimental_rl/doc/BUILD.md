# Build Instructions
### Steps
First, install all ROS2 dependencies from ROS2 Installation page: 

Second, install dependencies:
1. Install TensorFlow by following instructions from: [tensorflow](https://www.tensorflow.org/install/pip)
2. Install Keras: `sudo pip install keras`
3. Install `TurtleBot3` by following instructions from here: [TurtleBot3 ROS2 Packages](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/#setup) Section 15.1.1.3

4. TurtleBot3 installation also installs gazebo-ros-pkgs from ROS2 branch.  However, at this time the needed services are not available on the ROS2 branch.  To be able to use RL package, pull Crystal branch. 
5. ```sh
   cd ~/turtlebot3_ws/src/gazebo/gazebo_ros_pkgs
   git checkout crystal
   ```
6.  ```sh 
    cd <turtlebot3_ws> colcon build --symlink-install
    ```

###Build Experimental RL
```sh
cd <directory_ros2_ws> source install/setup.bash
cd <turtlebot3_ws> source install setup.bash
cd <navigation2_ws> colcon build --symlink-install

```
