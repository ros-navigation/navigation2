



﻿ROS2 Navigation - Turtlebot3 - June, 2019

# ROS 2 NAVIGATION 2 TUTORIAL

Tutorial Time: 2-5 hours
Difficulty Level: Beginner/Intermediate

 - [x] Overview
	 - What is ROS 2?
	 - What is Navigation 2?
	 - System Requirements
		 - Install VirtualBox (Optional)
		 - Install Ubuntu 18.04
 - [x]  ROS 2 Installation Steps
	 - Test ROS 2
 - [x]  Build Gazebo-ROS
 - [x]  Install Gazebo
	 - Test Gazebo with ROS
 - [x]  Navigation 2 Installation Steps
 - [x] Turtlebot 3 Installation Steps
	- Test Turtlebot 3 with Teleop Keyboard
 - [x]  Testing Navigation 2
	 - [x]  Testing Navigation 2 in Gazebo
	 - [ ]  Testing Navigation 3 with a real Turtlebot 3
		- [ ]  Setting up Turtlebot 3 Waffle
 - [ ]  Videos
 - [x]  More about Navigation 2
 - [ ]  Troubleshooting Guide
 - [x]  Useful Resources & Links

# Overview:

## What is ROS2?

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/Ros2_logo.jpg?raw=true)
###### Original Image Source: https://i.udemycdn.com/course/750x422/1797828_c391_3.jpg

The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project. And it’s all open source. In this tutorial we are going to use some of these ROS packages and tools. You can find more information about ROS2.
**[here](https://index.ros.org/doc/ros2/).**

*Please note that ROS2 is still under development, so it is important that you use the most up to date distribution and the repositories.*

## What is Navigation 2?

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/navigation2_logo.png?raw=true)
###### Original Image Source: https://avatars2.githubusercontent.com/u/2328631?s=400&v=4

The ROS 2 Navigation System is the control system that enables a robot to autonomously reach a goal state, such as a specific position and orientation relative to a specific map. Given a current pose, a map, and a goal, such as a destination pose, the navigation system generates a plan to reach the goal, and outputs commands to autonomously drive the robot, respecting any safety constraints and avoiding obstacles encountered along the way. More information about Navigation2 Stack can be found **[here](https://github.com/ros-planning/navigation2)**.

It is being actively developed and maintained by the Open Source Robotics Team at Intel.

*Please note that ROS2 is still in development stage.*

## What is Turtlebot 3?

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/tb3.jpg?raw=true)
##### Image:  Turtlebot 3 Waffle with some modifications.

In this tutorial we will learn how to control Turtlebot 3 using the Navigation2 stack. The navigation stack is being designed to work with most of the ROS2 wheeled robots, but in this tutorial we will only use Turtlebot 3. TurtleBot is a low-cost, personal robot kit with open-source software (ROS2). You can find more information about Turtlebot3 [here.](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

In this tutorial we will use Turtlebot Waffle robot in summation (Gazebo). We will also learn how to control Turtlebot Waffle in real life. Links to the robot kits:

 - [Turtlebot Burger](http://www.robotis.us/turtlebot-3-burger-us/)
 - [Turtlebot Waffle](http://www.robotis.us/turtlebot-3-waffle-pi/)

This tutorial may take about 2-5 hours to complete completely depends on your experience with ROS and robots and what computer system you have.

## System Requirements:

There is no minimum system requirement for this tutorial. A better system will help you build ROS2 packages faster and is going to deliver better performance when you run multiple tasks at the same time. This tutorial should work without any issues as long as you have a "newer" PC (less than 5 years old) with at least Intel i5 processor and 8 GB RAM. You will need some free disk space to install all the necessary software. I recommend you have at least 10 GB free disk space. 

### Hardware/Operating System

We tested this tutorial using the following systems:

**System 1:**
 - [NUC - NUCH8SV](https://www.intel.com/content/www/us/en/products/docs/boards-kits/nuc/mini-pcs/nuc-8-vr.html)
	 - CPU: i7-8809G
	 - GPU: Radeon RX Vega M GH graphics
	 - 32 GB DDR4-2400 SO-DIMM RAM
 - Operating System:
	 - Ubuntu 18.04
	 
**System2:**
- [ThinkPad T480](https://www.lenovo.com/us/en/laptops/thinkpad/thinkpad-t-series/ThinkPad-T480/p/22TP2TT4800?gclid=EAIaIQobChMIq9b80sqA4wIVCMNkCh1H5gG5EAAYASAAEgINKfD_BwE&cid=us:sem%7Cse%7Cgoogle%7C415787021890%7CThinkPad%20T480%7CIIP_NX_Thinkpad%20T%20Series_SMB%7C196131224&s_kwcid=AL!4030!3!343437426873!e!!g!!thinkpad%20t480&kw=thinkpad%20t480&adid=343437426873&addisttype=g&ef_id=EAIaIQobChMIq9b80sqA4wIVCMNkCh1H5gG5EAAYASAAEgINKfD_BwE:G:s&s_kwcid=AL!4030!3!343437426873!e!!g!!thinkpad%20t480)
	- CPU i5 vPro (8gen) 1.7 Ghz
	- 8 GB Ram
- Operating System: Windows 10
- Oracle VM VirtualBox
- Ubuntu 18.04

### Install Virtual Box (Optional)

If you only want to try this tutorial out without chaining your PC's operating system, you can create a Virtual Machine and Install Ubuntu 18.04 on it. Just we aware of that if you pick this method building packages is going to take much longer time. You can download Virtual Box [here](https://www.virtualbox.org/).  If you have never done this before, there are a lot of good examples on the Internet. You can search them [here](https://www.google.com/search?q=Install%20Ubuntu%2018.04%20in%20Virtualbox).

### Install Ubuntu 18.04

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/ubuntu_logo.jpeg?raw=true)
######  Original Image Source:  https://www.ubuntufree.com/wp-content/uploads/2018/04/Ubuntu-bionic-beaver-official-logo-2018-e1524742194287-1024x473.jpeg

Install the Desktop version of 18.04 

 - http://releases.ubuntu.com/18.04/

If you have never installed Ubuntu before, you can find the instructions here.

 - [How to install Ubuntu?](http://releases.ubuntu.com/18.04/)

# ROS2 Installation Steps

After installing Ubuntu 18.04 and connecting it to the Internet, now it is time to install ROS2. You can do it by following the instructions below.

There is a good chance that the installation steps for ROS2 will change over time. You want to check the installation steps at [this link](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup) first. If there is any difference, you should follow those steps instead.

**Set Locale**

    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    
**Setup Sources and add the repository to your sources list:**

    sudo apt update && sudo apt install curl gnupg2 lsb-release
    curl http://repo.ros2.org/repos.key | sudo apt-key add -
    sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

**Install Development Tools and ROS2 tools**

    sudo apt update && sudo apt install -y \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      python3-lark-parser \
      python3-pip \
      python-rosdep \
      python3-vcstool \
      wget

**Install some pip packages needed for testing**

    python3 -m pip install -U \
      argcomplete \
      flake8 \
      flake8-blind-except \
      flake8-builtins \
      flake8-class-newline \
      flake8-comprehensions \
      flake8-deprecated \
      flake8-docstrings \
      flake8-import-order \
      flake8-quotes \
      pytest-repeat \
      pytest-rerunfailures \
      pytest \
      pytest-cov \
      pytest-runner \
      setuptools
*In this step, you may get some warning messages, but don't worry as long as you don't get any error messages, you can ignore the warning massages for now*

**Install Fast-RTPS dependencies**

    sudo apt install --no-install-recommends -y \
      libasio-dev \
      libtinyxml2-dev

During this tutorial, we are going to build a lot of packages from source, so let's create a parent workspace that is going to be a home for all the other sub workspaces that we will created later in this tutorial. I will call this work space ros2_all_ws however you can call it what ever you like. 

*I recommend you using the same names that I use, so you can copy and past all the fallowing commands without any changes.*
	
**Create a new directory for our parent workspace**
	
    mkdir ros2_all_ws

**Create a new workspace for ROS2 in the parent workspace**
	
    cd ros2_all_ws
    mkdir ros2_ws
    cd ros2_ws
    mkdir src

**Clone all the up to date repositories in ros2_ws**

    wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos

*Note: If you want to get all of the latest bug fixes then you can try the “tip” of development by replacing **release-latest** in the URL above with **master**. The **release-latest** is preferred by default because it goes through more rigorous testing on release than changes to master do.* 
In this tutorial we will be using the **master**.

    vcs import src < ros2.repos
    sudo rosdep init
    rosdep update

*Note*: If `sudo rosdep init` doesn't work, use `sudo -E rosdep init`.

*If you see an error message saying "default source list file already exists", you need to remove that file first and run `rosdep init` again. You can delete the file by typing,

    sudo rm -rf /etc/ros/rosdep/source.list.d/*

**Install dependencies using rosdep**

    rosdep install --from-paths src --ignore-src --rosdistro crystal -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"
    
    python3 -m pip install -U lark-parser

**Build ROS2**

    colcon build --symlink-install

or

    colcon build --symlink-install --merge-install

*Note*: I recommend you use --merge-install option since it will merge all the setup.bash files. Currently, sourcing takes a bit longer than usual, so this may save you some time when you open a new terminal.

## Test ROS2

We are done building ROS2. Now, it is time to test it!

The easiest way to do it is using the ROS's famous talker and listener examples. Let's run the talker node first. What the talker does is publishing messages ("Hello World") to a topic. We will use the listener node to subscribe to the same topic to get these published messages and print them in the terminal.

Open a new terminal and run (this node is written in C++)

    source ~/ros2_all_ws/ros2_ws/install/local_setup.bash
    ros2 run demo_nodes_cpp talker

Open another terminal and run (this node is written in Python)

    source ~/ros2_all_ws/ros2_ws/install/local_setup.bash
    ros2 run demo_nodes_py listener
   
After running both nodes, you should see results similar to this.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/ros2_test.PNG?raw=true)

#  Gazebo Simulator and ROS2-Gazebo Installation

We will be using Gazebo 9 with ROS 2, so we need to install the Gazebo packages for ROS2. The following command is going to install all the necessary ros-gazebo packages for ROS Dashing distribution and Gazebo 9.

    sudo apt-get install ros-dashing-gazebo*

## Test Gazebo

Let's test ROS2-Gazebo and Gazebo together.

    sudo apt install ros-dashing-ros-core ros-dashing-geometry2
    gazebo --verbose /opt/ros/dashing/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world

If Gazebo prints "plugin can't be found" error message, try running it again

    source ~/ros2_all_ws/ros2_ws/install/setup.bash

Once Gazebo is launched, you should see a similar result. Now let's publish to a topic to move our robot.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/Screenshot%20from%202019-06-27%2010-08-56.png?raw=true)

Publishing this message to cmd_demo will move the robot forward.
   
     ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1

You should see our car moving forward. That means everything is working as expected.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/gazebo_test.gif?raw=true)

# Navigation 2 Installation Steps

Now, it is time to download and build Navigation2 stack. Stack means that Navigation2 consists of many other packages. By building the stack we will build all this necessary packages for Navigation 2 at once. Please note that we are going to test Navigation2 after building the Turtlebot3 stack.

**Install the dependencies for Navigation 2**

    sudo apt-get install -y \
        libsdl-image1.2 \
        libsdl-image1.2-dev \
        libsdl1.2debian \
        libsdl1.2-dev

**Create work spaces for Navigation2 and NavStack Dependencies**

    cd ~/ros2_all_ws
    mkdir -p /navstack_dependencies_ws/src
    mkdir -p /nav2_ws/src

**Clone  and build Navigation 2 Dependencies**

    wget https://raw.githubusercontent.com/ros-planning/navigation2/master/tools/ros2_dependencies.repos
    vcs import src < ros2_dependencies.repos
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro dashing --skip-keys "catkin"
    source ~/ros2_all_ws/ros2_ws/install/setup.bash
    colcon build --symlink-install

**Clone  and build Navigation 2**

	cd ~/ros2_all_ws/nav2_ws/src
	git clone https://github.com/ros-planning/navigation2.git
    cd ~/ros2_all_ws/nav2_ws
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro dashing
    source ~/ros2_all_ws/ros2_ws/install/setup.bash
    source ~/ros2_all_ws/navstack_dependencies_ws/install/setup.bash
    colcon build --symlink-install

## ROS 2 Turtlebot 3 Installation

It is time to download and build Turtlebot 3 packages.

**Install Cartographer dependencies**

    $ sudo apt install -y \
        google-mock \
        libceres-dev \
        liblua5.3-dev \
        libboost-dev \
        libboost-iostreams-dev \
        libprotobuf-dev \
        protobuf-compiler \
        libcairo2-dev \
        libpcl-dev \
        python3-sphinx

Cartographer package is used for creating maps. You can find more information about it [here](https://github.com/ros2/cartographer). We won't use it in this tutorial.

**Install Navigation2 dependencies**

    $ sudo apt install -y \
        libsdl-image1.2 \
        libsdl-image1.2-dev \
        libsdl1.2debian \
        libsdl1.2-dev

In this step, we won't build the Navigation 2 stack, but we still need these dependencies. We will build Navigation2 separately.

**Create a workspace for Turtlebot 3 package**
  
      cd ~/ros2_all_ws
      mkdir turtlebot3_ws
      cd turtlebot3_ws
      mkdir src

**Clone all the up to date repositories**

    wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
    vcs import src < turtlebot3.repos

**Important:** You must remove Navigation2 packages from the Turtlebot3 workspace before you build it. Because we will download and build Navigation2 stack separately.

    cd ~/ros2_all_ws/turtlebot3_ws/src
    sudo rm -r navigation2
    cd ~/ros2_all_ws/turtlebot3_ws
    
**Build the package**

we should source the preciously installed packages first,
	
    source ~/ros2_all_ws/ros2_ws/install/setup.bash
    source ~/ros2_all_ws/navigation2_ws/install/setup.bash
    source ~/ros2_all_ws/navstack_dependencies_ws/install/setup.bash    

build the package,

    colcon build --symlink-install

If you see some warning messages  such as "package A had stdeer output", as long as all the packages are successfully built you can ignore those warning messages.

if you see any dependency related error messages, you can try to run rosdep and try to build the package again,

	rosdep install --from-paths src --ignore-src -r -y

### Test Turtlebot 3

Before executing any nodes, we need to specify which Turtlebot3 model we want to use. I will use Turtlebot Waffle for this tutorial. In the fallowing steps we will use the export command to set this variable. Eventually, this line will end up in our .bashrc file, but for now we will run it manually.
 
    export TURTLEBOT3_MODEL=waffle

There are three possible options.
 - burger
 - waffle
 - waffle-pi

Before we launch anything, we need to source the each setup.bash in every workspace that we created. Eventually, we source all of them in the ~./bashrc file, but for now we will only use the terminals. At this step we want to keep our .bashrc file clean. In the next step we will learn how to make this task automated.

Let's open a new terminal and type,

    export TURTLEBOT3_MODEL=waffle
    source ~/ros2_all_ws/ros2_ws/install/setup.bash
    source ~/ros2_all_ws/turtlebot3_ws/install/setup.bash
    source ~/ros2_all_ws/navstack_dependencies_ws/install/setup.bash
    source ~/ros2_all_ws/navigation2_ws/install/setup.bash
    ros2 run turtlebot3_teleop teleop_keyboard

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/teleop.PNG?raw=true)
This will start the teleoperation node, so we can use our keyboard to control the linear and angular velocity of our Turtlebot. This node will publish to /cmd_vel. You can also use ros2 pub to publish to the same topic and control the robot. For example, to move the robot forward, you can publish the fallowing message to the /cmd_vel topic.

    ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}}' -1

To make things simpler, we will use the keyboard.

    export TURTLEBOT3_MODEL=waffle
    source ~/ros2_all_ws/ros2_ws/install/setup.bash
    source ~/ros2_all_ws/turtlebot3_ws/install/setup.bash
    source ~/ros2_all_ws/navstack_dependencies_ws/install/setup.bash
    source ~/ros2_all_ws/navigation2_ws/install/setup.bash
    ros2 launch turtlebot3_bringup robot.launch.py

This will run the bringup launch file for Turtlebot. It automatically will set all the necessary parameters and run all the necessary nodes that we will use to control our Turtlebot.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/turtlebot_bringup.PNG?raw=true)
Let's open another terminal and type

    export TURTLEBOT3_MODEL=waffle
    source ~/ros2_all_ws/ros2_ws/install/setup.bash
    source ~/ros2_all_ws/turtlebot3_ws/install/setup.bash
    source ~/ros2_all_ws/navstack_dependencies_ws/install/setup.bash
    source ~/ros2_all_ws/navigation2_ws/install/setup.bash
    ros2 launch turtlebot3_bringup rviz2.launch.py

This will launch RVIZ  (Robot Visualization Tool) with some saved settings RVIZ will help us see what our robot's state in other words all its sensor values. For now, we won't use RVIZ much, but we will use RVIZ to test the Navigation stack, so it is good to know that RVIZ works fine. 

Open one last terminal and type
	
    source ~/ros2_all_ws/ros2_ws/install/setup.bash
    source ~/ros2_all_ws/turtlebot3_ws/install/setup.bash
    source ~/ros2_all_ws/navstack_dependencies_ws/install/setup.bash
    source ~/ros2_all_ws/navigation2_ws/install/setup.bash

Tell Gazebo where to find the world and robot models. We need to set some environment variables first and then launch Gazebo with Turtlebot and it's Turtle word model.

    echo '# Add gazebo model path' >> ~/.bashrc
    echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_all_ws/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
    source ~/.bashrc
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


![enter image description here](https://github.com/mlherd/ros2/blob/master/img/gazebo_turlebot1.PNG?raw=true)

This will start Gazebo and load the world model and robot model which means that we will see a 3D simulated world and simulation of our Turtlebot Waffle.

Now, you should be able control the Turtlebot with your keyboard buttons. Make sure you are the keyboard teleop node terminal when pressing the buttons to control the Turtlebot. Otherwise, Turtlebot won't receive any velocity commands.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/gazebo_turtlebot2.PNG?raw=true)

*Note: After using source you may experience some delay in the terminal (some times a few minutes). No worries, it is normal.*

# Testing Navigation 2

## Testing Navigation 2 in Gazebo

Now, we know all the packages and ROS tools we need to use the Navigation stack work, so it is time to test Navigation 2. Before we do anything, in order to save time, let's export all the source commands and environment variables to the .bashrc file. 

Open a new terminal and open your .bashrc file which is located in your home directory. You can use any one of your favorite text editors, just remember to have  super user rights to be able to modify the file. I will use gedit to edit the file.

    sudo gedit ~/.bashrc

Add the following lines to the end of the file

    source ~/ros2_all_ws/ros2_ws/install/setup.bash  
    source ~/ros2_all_ws/turtlebot3/install/setup.bash
    source ~/ros2_all_ws/navigation2/install/setup.bash
    source ~/ros2_all_ws/navstack_dependencies_ws/install/setup.bash
    export TURTLEBOT3_MODEL=waffle
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_all_ws/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models

save the file and close it. Whenever we open a new terminal, this script will be executed. Please note that this will will make your terminal's start up time considerably longer, so when you are done with this tutorial you may want to remove these lines from your .bashrc file.

Let's launch Gazebo with the world model first, open a new terminal and type

    gazebo --verbose -s libgazebo_ros_init.so /home/<user_name>/ros2_all_ws/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_worlds/waffle.model

Don't forget to change <user_name> to your actual user name. For example, after typing your user name it should look like this,

    gazebo --verbose -s libgazebo_ros_init.so /home/melih/ros2_all_ws/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_worlds/waffle.model


![enter image description here](https://github.com/mlherd/ros2/blob/master/img/gazebo_turlebot1.PNG?raw=true)

Next we will start the Turtlebot specific nodes,

    ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py use_sim_time:=True

It is time to launch the Navigation 2 stack

    ros2 launch nav2_bringup nav2_bringup_launch.py use_sim_time:=True autostart:=False \
    map:=/home/<user_name>/ros2_all_ws/navigation2_ws/install/nav2_bringup/share/nav2_bringup/launch/turtlebot3_world.yaml

You can change autostart:=False to autostart:=True.

Again don't forget to change <user_name> to your actual user name. For example, after typing your user name it should look like this,

    ros2 launch nav2_bringup nav2_bringup_launch.py use_sim_time:=True autostart:=False \
        map:=/home/melih/ros2_all_ws/navigation2_ws/install/nav2_bringup/share/nav2_bringup/launch/turtlebot3_world.yaml

Lastly, we will launch RVIZ with a configuration file for Turlebot. 

    ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/nav2_default_view.rviz

Now, you should start seeing the Turtlebot in the center of the plot in Rviz. Click on the Startup button (Bottom Left)

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/Rviz_start.PNG?raw=true)

Now, the map should appear in Rviz. Next we need to set the estimate 2D Position of our Turtlebot on the map. Looking at the simulator in Gazebo we know that our Turtlebot is located at the bottom right in the map.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/Rviz_initial.PNG?raw=true)

Let's click on the 2D Pose Estimate button and point the location of our robot on the map. The direction of the green arrow is the orientation of Turtlebot.

    -  Localize the robot using “2D Pose Estimate” button.
    -  Make sure all transforms from odom are present. (odom->base_link->base_scan)

First, find where the robot is in Gazebo. You can see where the robot's initial position in Gazebo.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/gazebo_turlebot1.PNG?raw=true)
Then set the pose of the robot in Rviz.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/Rviz2D_Pose_Estimate.PNG?raw=true)
Now, the 3D model of Turtlebot should move to that location. A small error in estimated location is OK.

Next, we will pick a target location for Turtlebot in the map.  We can send our robot a goal position and a goal orientation by using the **Navigation2 Goal** button.

*Note*: this uses a ROS2 Action to send the goal, and a pop-up window will appear on your screen with a 'cancel' button if you wish to cancel.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/RvizNavigation2Goal1.PNG?raw=true)

Once we define the target location, Turtlebot will create a global path and start navigating in the map.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/RvizNavigation2Goal2.PNG?raw=true)
You can also observe that our Turtlebot move in the simulated environment in Gazebo as well.

![enter image description here](https://github.com/mlherd/ros2/blob/master/img/gazebo_turtlebot3.PNG?raw=true)
This steps concludes the first part of this tutorial. In the fallowing sections, we are going to learn how to control the real Turtlebot.

![enter image description here](https://raw.githubusercontent.com/mlherd/ros2/master/img/nav2_gif.gif)

## Testing Navigation 2 with a real Turtlebot 3
- This step soon to be completed. We are waiting for Turtlebot 3 firmware release for Dashing.
###  Setting Up Turtlebot Waffle
- You can follow the instruction [here](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/).  Complete following steps on Turtlebot3.
	- 1.2
	- 1.3
	- 2.1
####  Test Navigation2 with the real Turtlebot Waffle**
Coming Soon!

## More About Navigation 2

![enter image description here](https://raw.githubusercontent.com/mlherd/ros2/master/img/nav2_uml_diagram.jpg)
**What does each individual module in Navigation 2 do?**

**RVIZ:** RVIZ is ROS's Robot Virtualization tool. As you see in the UML diagram above we use it to generate the Goal Pose messages which is received by the BTNAV node. RVIZ also shows us all the sensor data and the map. You can find more information about RVIZ [here](http://wiki.ros.org/rviz).

**BTNAV:** The BT Navigator receives a goal pose and navigates the robot to the specified destination. In this case it receives the goal pose message from RVIZ. It also uses a Behavior Tree which is described in an XLM format. Behavior Tree can be customized and is used to decide how and in which order the navigation task will be completed. More information about this module can be found [here](https://github.com/ros-planning/navigation2/tree/master/nav2_bt_navigator).

**NAVFN:** NAVFN is a planning module. It creates plans for the robot. It assumes a circular robot. The planner uses start and end points to find a minimum cost plan using a costmap.  A* and Dijkstra's search algorithms are used to find the shortest path. You can choose either method.  The planning algorithm based on the [DWA](https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf). You can find more information about this module [here](https://github.com/ros-planning/navigation2/tree/master/nav2_navfn_planner).

**DWB:** This module is used for local robot navigation. It provides a controller that drives the robot in the plane. The source code and documentation for the DWB module can be found [here](https://github.com/ros-planning/navigation2/tree/master/nav2_dwb_controller).

**World Model:** World model generates cost map which is an occupancy grid created by using sensor data such as lidar sensor data. You can find the source code and more information about World Model module [here](https://github.com/ros-planning/navigation2/tree/master/nav2_world_model).

**Gazebo:** Gazebo is the simulator.

**How everything is connected?**

During this tutorial we launched many nodes. If you want to see what these nodes publish and how they communicate with each other, you can do it with the rqt_gui tool. 

    ros2 run rqt_gui rqt_gui

![enter image description here](https://raw.githubusercontent.com/mlherd/ros2/master/img/rqt_graph.png)

# Results

![enter image description here](https://raw.githubusercontent.com/mlherd/ros2/master/img/nav2_gif.gif)
# Troubleshooting Guide
-To do: a list of common issues and problems.
- Gazebo crashes or unable to start server - Address is already in use
	- `killall gzserver`
	- restart Gazebo
	- repeat this until it successfully starts

# Resources & Useful Links

- [https://github.com/ros-planning/navigation2](https://github.com/ros-planning/navigation2)
 - [https://github.com/ros-planning/navigation2/tree/master/nav2_bringup](https://github.com/ros-planning/navigation2/tree/master/nav2_bringup)
 - [http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/)
 - [https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Development-Setup/](https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Development-Setup/)
 - [https://github.com/ros2/cartographer](https://github.com/ros2/cartographer)
