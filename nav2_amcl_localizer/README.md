# AMCL
Adaptive Monte Carlo Localization (AMCL) is a probabilistic localization module which estimates the position and orientation (i.e. Pose) of a robot in a given known map.

## Overview
Currently, the AMCL module in ROS 2 Navigation System is a direct port from ROS1 AMCL package (http://wiki.ros.org/amcl) with some minor code re-factoring. The direct port includes all of ROS1 functionalities except dynamic parameter reconfiguration and running from Bag files. The code re-factoring includes moving Sensor, Particle Filter, and Map from nav2_amcl directory to nav2_util directory. The main logic behind this is code re-usability.  However, given the way in which these classes are currently written, these codes are highly bounded to AMCL.  Therefore, to make use of code re-usability, these classes need to be substantially modified.

## Current Plan
* Polishing AMCL core code, specially the `laserReceived` callback [Issue 211](https://github.com/ros-planning/navigation2/issues/211)
* Refactoring redundant codes such as `angle_diff` which exist in multiple places [Issue 210](https://github.com/ros-planning/navigation2/issues/210)
* Using generic Particle Filter library [Issue 206](https://github.com/ros-planning/navigation2/issues/206)
* Creating a generic library to pull out the algorithms that are from Probabilistic Robotics textbook.  These algorithms could potentially be used on other modules [Issue 207](https://github.com/ros-planning/navigation2/issues/207)
* Enabling dynamic reconfigure functionality [Issue 209](https://github.com/ros-planning/navigation2/issues/209)

## Future Plan
* Running from Ros bag 
* Extending AMCL to work with different type of Sensors
* Improving overall localization performance with fusing data from different sensors such as IMU, Sonar, Radar, Laser, Depth camera, and etc.
