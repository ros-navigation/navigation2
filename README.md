# Navigation2

ROS2 Navigation System

[![Build Status](https://circleci.com/gh/ros-planning/navigation2/tree/master.svg?style=svg)](https://circleci.com/gh/ros-planning/navigation2/tree/master) CircleCI

[![Build Status](https://img.shields.io/docker/cloud/build/rosplanning/navigation2.svg?label=build)](https://hub.docker.com/r/rosplanning/navigation2) DockerHub

[![Build Status](http://build.ros2.org/job/Cdev__navigation2__ubuntu_bionic_amd64/badge/icon)](http://build.ros2.org/job/Cdev__navigation2__ubuntu_bionic_amd64/) ROS Build Farm 

[![codecov](https://codecov.io/gh/ros-planning/navigation2/branch/master/graph/badge.svg)](https://codecov.io/gh/ros-planning/navigation2)

# Overview
The ROS 2 Navigation System is the control system that enables a robot to autonomously reach a goal state, such as a specific position and orientation relative to a specific map. Given a current pose, a map, and a goal, such as a destination pose, the navigation system generates a plan to reach the goal, and outputs commands to autonomously drive the robot, respecting any safety constraints and avoiding obstacles encountered along the way.

![nav2_overview](doc/architecture/navigation_overview.png)

# Documentation
For detailed instructions on how to install and run the examples, please visit our [documentation site](https://ros-planning.github.io/navigation2/).

# Contributing
[Contributions are welcome!](doc/README.md#contributing). For more information, please review our [contribution guidelines](https://ros-planning.github.io/navigation2/contribute/contribute_guidelines.html).

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

## Using CI build docker container

We allow for you to pull the latest docker image from the master branch at any time. As new releases and tags are made, docker containers on docker hub will be versioned as well to chose from.

```
sudo docker pull rosplanning/navigation2:latest
```
