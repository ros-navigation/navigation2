ROS2 Navigation2 - Turtlebot3 - Oct, 2019
# Navigation2 - Turtlebot 3

- [Overview](#overview)
- [Requirments](#requirements)
- [Navigation2 with Turtlebot 3 in Gazebo](#navigation2-with-turtlebot-3-in-gazebo)
- [Navigation2 with a Real Turtlebot 3](#navigation2-with-a-real-turtlebot-3)
- [Videos](#videos)

# Overview:

This tutorial shows how to control and navigate Turtlebot 3 using the ROS2 Navigation2. Turtlebot is a low-cost, personal robot kit with open-source software (ROS2). Turtblebot robots are widely supported by the ROS community. You can find many applications and examples of Turtlebot projects on the Internet. You can find more information about Turtlebot3 [here.](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

Links to the robot kits:

- [Turtlebot Burger](http://www.robotis.us/turtlebot-3-burger-us/)
- [Turtlebot Waffle](http://www.robotis.us/turtlebot-3-waffle-pi/)

This tutorial consists of two parts. In the first part, you will learn how to use Turtlebot 3 robots in summation (Gazebo). And in the second part, you will learn how to control a real Turtlebot Waffle using Navigation2.

```ROS2 Dashing``` and ```Navigation2 Dashing 0.2.4``` are used to create this tutorial.

This tutorial may take about 2-5 hours to complete completely depends on your experience with ROS, robots and what computer system you have.

## Requirements:

- [Install ROS2]([https://index.ros.org/doc/ros2/Installation/](https://index.ros.org/doc/ros2/Installation/))

- Install Navigation2

    - ```sudo apt install ros-<ros2-distro>-navigation2 ros-<ros2-distro>-nav2-bringup```

- Install Turtlebot3 

    - ```sudo apt install ros-<ros2-distro>-turtlebot3*```

- [Setup Turtlbot 3](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/#setup)

- [Install Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

## Navigation2 with Turtlebot 3 in Gazebo

### 0- Setup Your Enviroment Variables

There are two ways to this.

**- Option 1**  Run the following commands whenever you open a new terminal during this tutorial. If you are a beginner level ROS user or for any reason you don't don't want to do any perminant changes in your enviroment, this option is higly recommended. 

- ```source /opt/ros/<ros2-distro>/setup.bash```
- ```export TURTLEBOT3_MODEL=waffle```
- ```export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/dashing/share/turtlebot3_gazebo/models```

**- Option 2** Edit your .bashrc file.

You need to make sure you know what you are doing in this step, otherwise, these changes could cause issues especially, if you already have ROS1 or other ROS2 distributions installed on your system and you already edited your .bashrc file.

Open a new terminal and open your .bashrc file which is located in your home directory. You can use your favorite text editor.

```
sudo gedit ~/.bashrc
```

Add the following lines to the end of the file.

```
source /opt/ros/<ros2-distro>/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/dashing/share/turtlebot3_gazebo/models
```
save the file and close it. Whenever we open a new terminal, this script will be executed. 

*Please note that this will make your terminal's start-up time considerably longer, so when you are done with this tutorial you may want to remove these lines from your .bashrc file.*

### 1- Launch Gazebo

Now, launch Gazebo with the world model, open a new terminal and type

    gazebo --verbose -s libgazebo_ros_init.so /opt/ros/dashing/share/turtlebot3_gazebo/worlds/turtlebot3_worlds/waffle.model

Once, Gazebo is launched, you should see the Turtlebot3 world and Turtlebot 3 Waffle.

If Gazebo fails to start, run the following commands and try to launch Gazebo again.

    killall gzserver
    killall gzclient

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/gazebo_turlebot1.PNG?raw=true)

### 2- Launch Turtlebot 3 Robot State Publisher
 
Launch Turtlebot 3 specific nodes,

    ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py use_sim_time:=True

Make sure use_sim_time is set to **True**.

### 3- Launch Navigation2 

Launch Navigation 2. If you set `autostart:=False`, you need to click on the start button in RVIZ to initialize the nodes. Make sure `use_sim time` is set to **True**, because we want to use the time simulation time in Gazebo instead of the system time.

turtlebot3_world.yaml is the configuration file for the map we want to provide Navigation2. In this case, it has the map resolution value, threshold values for obstacles and free spaces, and a map file location.

```
ros2 launch nav2_bringup nav2_bringup_launch.py use_sim_time:=True autostart:=False map:=/opt/ros/dashing/share/nav2_bringup/launch/turtlebot3_world.yaml
```

### 4-  Launch RVIZ

Launch RVIZ with a pre-defined configuration file.

    ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/nav2_default_view.rviz

Now, you should see a shadow of Turtlebot 3 robot model in the center of the plot in Rviz. Click on the Start button (Bottom Left) if you set the auto_start parameter to false.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/Rviz_start.PNG?raw=true)

Now, the map should appear in Rviz.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/Rviz_initial.PNG?raw=true)

### 5- Initialize the location of Turtlebot 3

First, find where the robot is in Gazebo. You can see where the robot's initial position in Gazebo.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/gazebo_turlebot1.PNG?raw=true)

Set the pose of the robot in Rviz. Click on the 2D Pose Estimate button and point the location of the robot on the map. The direction of the green arrow is the orientation of Turtlebot.
  
![enter image description here](https://github.com/mlherd/ros2/blob/master/img/Rviz2D_Pose_Estimate.PNG?raw=true)

Now, the 3D model of Turtlebot should move to that location. A small error in the estimated location is tolerable.

### 6-  Send a Goal Pose

Pick a target location for Turtlebot on the map. You can send Turtlebot 3 a goal position and a goal orientation by using the **Navigation2 Goal** the **GoalTool** buttons.

*Note*: Difference between buttons. Navigation2 Goal button uses a ROS2 Action to send the goal and GoalTool publishes the goal to a topic.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/RvizNavigation2Goal1.PNG?raw=true)

Once you define the target pose,  Navigation2 will find a global path and start navigating the robot on the map.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/RvizNavigation2Goal2.PNG?raw=true)

You can also observe that Turtlebot 3 moves in the simulated environment in Gazebo as well.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/gazebo_turtlebot3.PNG?raw=true)

This step concludes the first part of this tutorial. In the following sections, you are going to learn how to control the real Turtlebot.

![enter image description here](https://raw.githubusercontent.com/mlherd/ros2/master/img/nav2_gif.gif)


## Navigation2 with a Real Turtlebot 3

### 0- Setup Your Enviroment Variables

There are two ways to this.

**- Option 1**  Run the following commands whenever you open a new terminal during this tutorial. If you are a beginner level ROS user or for any reason you don't don't want to do any perminant changes in your enviroment, this option is higly recommended. 

- ```source /opt/ros/<ros2-distro>/setup.bash```
- ```export TURTLEBOT3_MODEL=waffle```

**- Option 2** Edit your .bashrc file.

You need to make sure you know what you are doing in this step, otherwise, these changes could cause issues especially, if you already have ROS 1 or other ROS 2 distributions installed on your system and you already eddited your .bashrc file.

Open a new terminal and open your .bashrc file which is located in your home directory. You can use your favorite text editor.

```
sudo gedit ~/.bashrc
```

Add the following lines to the end of the file.

```
source /opt/ros/<ros2-distro>/setup.bash
export TURTLEBOT3_MODEL=waffle
```
save the file and close it. Whenever we open a new terminal, this script will be executed. 

*Please note that this will  make your terminal's start-up time considerably longer, so when you are done with this tutorial you may want to remove these lines from your .bashrc file.*

### 1- Launch Turtlebot 3 Robot State Publisher

Launch Turtlebot 3 robot state publisher,

    ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py use_sim_time:=True

Make sure use_sim_time is set to **False**.

### 2- Launch Navigation2 
 
 You need to have a map of the environment where you want to Navigate Turtlebot 3. Required files:

   - ```your-map.map```
   - ```your-map.yaml```

`<your_map>.yaml` is the configuration file for the map we want to provide Navigation2. In this case, it has the map resolution value, threshold values for obstacles and free spaces, and a map file location. You need to make sure these values are correct. More information about the map.yaml can be found [here](http://wiki.ros.org/map_server).

Launch Navigation 2. If you set autostart:=False, you need to click on the start button in RVIZ to initialize the nodes. Make sure `use_sim time` is set to **False**, because we want to use the system time instead of the time simulation time in Gazebo.

```
ros2 launch nav2_bringup nav2_bringup_launch.py use_sim_time:=False autostart:=False map:=/path/to/your-map.yaml
```
Don't forget to change **/path/to/your-map.yaml** to the actual path to the your-map.yaml file.

### 3-  Launch RVIZ

Launch RVIZ with a pre-defined configuration file.

    ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/nav2_default_view.rviz

Now, you should see a shadow of Turtlebot 3 robot model in the center of the plot in Rviz. Click on the Start button (Bottom Left) if you set the auto_start parameter to false.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/Rviz_start.PNG?raw=true)

Now, the map should appear in Rviz.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/rviz_custom_map.png?raw=true)


### 4- Initialize the Location of Turtlebot 3

First, find where the robot is on the map. Check where your robot is in the room.

Set the pose of the robot in Rviz. Click on the 2D Pose Estimate button and point the location of the robot on the map. The direction of the green arrow is the orientation of Turtlebot.
  
![enter image description here](https://github.com/mlherd/ros2/blob/master/img/rviz_2d_pose_extimate_custom_map.png?raw=true)

Now, the 3D model of Turtlebot should move to that location. A small error in the estimated location is tolerable.

### 5-  Send a Goal Pose

Pick a target location for Turtlebot on the map. You can send Turtlebot 3 a goal position and a goal orientation by using the **Navigation2 Goal** the **GoalTool** buttons.

*Note*: Difference between buttons. Navigation2 Goal button uses a ROS2 Action to send the goal and GoalTool publishes the goal to a topic.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/rviz_send_goal_pose_custom_map.png?raw=true)

Once you define the target pose,  Navigation2 will find a global path and start navigating the robot on the map.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/rviz_navigate_custom_map.png?raw=true)

Now, you can see that Turtlebot 3 moves towards the goal position in the room. See the video below.

These steps conclude this tutorial part of this tutorial.

## Videos

*TODO: Add videos.*
