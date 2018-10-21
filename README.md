# Navigation2

ROS2 Navigation System

[![Build Status](https://travis-ci.org/ros-planning/navigation2.svg?branch=master)](https://travis-ci.org/ros-planning/navigation2)

# Overview
The ROS 2 Navigation System is the control system that enables a robot to autonomously reach a goal state, such as a specific position and orientation relative to a specific map. Given a current pose, a map, and a goal, such as a destination pose, the navigation system generates a plan to reach the goal, and outputs commands to autonomously drive the robot, respecting any safety constraints and avoiding obstacles encountered along the way.

# Contributing
We are currently in the pre-release development phase, contributions are welcome. To contribute, see the [documentation README](doc/README.md).

# Building the source
For instructions on how to download and build this repo, see the [BUILD.md](doc/BUILD.md) file.

# Creating a docker image
To build an image from the Dockerfile in the navigation2 folder: 
First, clone the repo to your local system (or see Building the source above)
```
sudo docker build -t nav2/latest .
```
If proxies are needed:
```
sudo docker build -t nav2/latest --build-arg http_proxy=http://proxy.my.com:### --build-arg https_proxy=http://proxy.my.com:### .
```
Note: You may also need to configure your docker for DNS to work. See article here for details:
https://development.robinwinslow.uk/2016/06/23/fix-docker-networking-dns/
