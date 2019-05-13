# Overview
The ROS 2 Experimental Reinfocement Learning (RL) package is a package that enables to train and control a robot with RL techniques.  This package is utilizing the Gazebo simulator and ROS 2 to train a TurtleBot 3 robot. 

# Contributing
We have currently implemented a Random Crawl task to get started with RL and plan to implement and add more RL related tasks to this package. This Random Crawl RL task can be used as a tutorial to get started with RL.

# Building the source
For instructions on how to build this repo, see the [BUILD.md](https://github.intel.com/haghighi/Experimental_RL/blob/master/doc/BUILD.md) file.

###Random Crawl
Random Crawl is a task which enables TurtleBot3 to randomly crawl around the environment without hitting any obstacles.
The following use-cases can be achieved with Random Crawl task:
1. Map Exploration
2. Localization test
3. Security Monitoring

#### Model
TODO
Inputs
Outputs
DQN
Parameters


####Training
To train Random Crawl:

Firtst run Gazebo:
```sh 
export <directory_for_turtlebot3_ws>/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=burger or waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Then, 
```sh
cd <navigation2_ws> source install setup.bash
```
Run:
```sh 
ros2 run nav2_turtlebot3_rl random_crawl_train
```

####Running
To run a trained model Random Crawl task:
Firtst run Gazebo:

```sh 
export <directory_for_turtlebot3_ws>/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=burger or waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Then, 
```sh
cd <navigation2_ws> source install setup.bash
```
Run:
```sh 
ros2 run nav2_turtlebot3_rl random_crawl
```
